#!/bin/bash
# Trail Stability: bauen, testen und installieren.
#   1. bauen   2. Selbsttest   3. nach /Library/OFX/Plugins installieren   4. Resolve-Plugin-Cache leeren
#
# Aufruf im Projektordner:   ./update.sh
# Option:   ./update.sh --sauber     (build-Ordner neu anlegen, z. B. nach einem Xcode-Update)

set -u
cd "$(cd "$(dirname "$0")" && pwd)" || exit 1
CLEAN=0
for a in "$@"; do
  case "$a" in
    --kein-zip) ;;   # alte Option, ohne Wirkung
    --sauber)   CLEAN=1 ;;
    *) echo "Unbekannte Option: $a"; exit 1 ;;
  esac
done

step() { printf '\n\033[1m== %s\033[0m\n' "$*"; }
fail() { printf '\n\033[31mFEHLER: %s\033[0m\n' "$*"; exit 1; }

# ---------------------------------------------------------------- 0. Voraussetzungen
step "Voraussetzungen"
command -v cmake >/dev/null || fail "cmake fehlt (brew install cmake)"
xcrun --find clang++ >/dev/null 2>&1 || fail "Compiler fehlt (xcode-select --install)"
echo "cmake: $(cmake --version | head -1)"
echo "Arch:  $(uname -m)"

if pgrep -x Resolve >/dev/null 2>&1; then
  echo
  echo "Hinweis: DaVinci Resolve läuft noch. Es lädt Plugins nur beim Start."
  echo "         Nach dem Update Resolve mit Cmd+Q beenden und neu starten."
  RESOLVE_WAS_RUNNING=1
else
  RESOLVE_WAS_RUNNING=0
fi

# ---------------------------------------------------------------- 1. Bauen
step "1/4 Bauen"
[ "$CLEAN" = 1 ] && { echo "build-Ordner wird neu angelegt"; rm -rf build; }
if [ ! -f build/CMakeCache.txt ]; then
  cmake -B build -DCMAKE_BUILD_TYPE=Release > build_cmake.log 2>&1 \
    || { tail -20 build_cmake.log; fail "cmake-Konfiguration fehlgeschlagen (Log: build_cmake.log)"; }
fi
cmake --build build -j "$(sysctl -n hw.ncpu)" 2>&1 | tee build_make.log | grep -E "error|Error|warning: unused|Linking|Built target Trail"
[ "${PIPESTATUS[0]}" = 0 ] || fail "Bauen fehlgeschlagen (Log: build_make.log). Bei 'file not found' für Standard-Header: ./update.sh --sauber"

# ---------------------------------------------------------------- 2. Selbsttest
step "2/4 Selbsttest"
./build/trailstab_selftest | tail -1 | grep -q "SELFTEST OK" || fail "Selbsttest nicht bestanden – es wird NICHT installiert."
echo "SELFTEST OK"

# ---------------------------------------------------------------- 3. Installieren
step "3/4 Installieren (Passwort wird gebraucht)"
sudo mkdir -p /Library/OFX/Plugins || fail "Ordner /Library/OFX/Plugins nicht anlegbar"
sudo cmake --install build > install.log 2>&1 || { tail -10 install.log; fail "Installation fehlgeschlagen (Log: install.log)"; }
# altes Diagnose-Plugin (bis v0.9 mitgebaut) entfernen
if [ -d /Library/OFX/Plugins/TrailStabProbe.ofx.bundle ]; then
  sudo rm -rf /Library/OFX/Plugins/TrailStabProbe.ofx.bundle && echo "altes TrailStab Probe entfernt."
fi
ls -d /Library/OFX/Plugins/TrailStab.ofx.bundle
# gegen Gatekeeper-Probleme bei selbst gebauten Plugins
for b in /Library/OFX/Plugins/TrailStab.ofx.bundle; do
  sudo xattr -cr "$b" 2>/dev/null
  sudo codesign --force --sign - "$b" >/dev/null 2>&1 || echo "Hinweis: codesign für $b übersprungen"
done

# ---------------------------------------------------------------- 4. Resolve-Cache
step "4/4 Resolve-Plugin-Cache"
CACHE="$HOME/Library/Application Support/Blackmagic Design/DaVinci Resolve/OFXPluginCacheV2.xml"
if [ "$RESOLVE_WAS_RUNNING" = 1 ]; then
  echo "Resolve läuft – Cache bleibt unangetastet. Erst Resolve beenden (Cmd+Q), dann:"
  echo "  rm \"$CACHE\""
  echo "(nur nötig, falls das Plugin nach dem Neustart nicht erscheint oder nicht aktualisiert ist)"
elif [ -f "$CACHE" ]; then
  rm -f "$CACHE" && echo "Cache gelöscht (Resolve baut ihn beim Start neu)."
else
  echo "kein Cache vorhanden."
fi

VER=$(strings /Library/OFX/Plugins/TrailStab.ofx.bundle/Contents/MacOS/TrailStab.ofx 2>/dev/null | grep -m1 -E "^v[0-9]+\.[0-9]+\.[0-9]+ \| Build")
printf '\n\033[32mFertig.\033[0m Installiert: %s\n' "${VER:-$(stat -f '%Sm' /Library/OFX/Plugins/TrailStab.ofx.bundle)}"
echo "Jetzt Resolve komplett beenden (Cmd+Q) und neu starten."
