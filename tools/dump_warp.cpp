// usage: dump_warp track.tstab smooth zoom -> per frame: i R00..R22 (virtual->input bearings)
#include "path_solver.h"
#include "sidecar.h"
#include <cstdio>
#include <cstdlib>
int main(int argc,char**argv){
    ts::CameraTrack tr; std::string e; if(!ts::loadTrack(argv[1],tr,&e)){fprintf(stderr,"%s\n",e.c_str());return 1;}
    ts::SolveParams p; p.smoothness=atof(argv[2]); p.zoom=atof(argv[3]); p.levelHorizon=argc<5||atoi(argv[4]); p.vertical=argc>5?atof(argv[5]):0.0;
    auto r=ts::solvePath(tr,p);
    fprintf(stderr,"status %d clamped %d maxcorr %.2f\n",r.status,r.clampedFrames,r.maxCorrectionDeg);
    for(size_t i=0;i<tr.size();++i){printf("%zu",i);for(int k=0;k<9;++k)printf(" %.9f",r.warp[i].m[k]);printf("\n");}
}
