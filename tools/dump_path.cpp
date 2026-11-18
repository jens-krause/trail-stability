// usage: dump_path track.tstab smooth zoom [vertical=1]  -> prints frame t E(yaw,pitch,roll) S(yaw,pitch,roll) in radians
#include "path_solver.h"
#include "sidecar.h"
#include <cstdio>
#include <cstdlib>
int main(int argc,char**argv){
    ts::CameraTrack tr; std::string e; if(!ts::loadTrack(argv[1],tr,&e)){fprintf(stderr,"%s\n",e.c_str());return 1;}
    ts::SolveParams p; p.smoothness=atof(argv[2]); p.zoom=atof(argv[3]); p.levelHorizon=true; p.vertical = argc>4 ? atof(argv[4]) : 1.0;
    auto r=ts::solvePath(tr,p);
    fprintf(stderr,"status %d clamped %d maxcorr %.2f\n",r.status,r.clampedFrames,r.maxCorrectionDeg);
    for(size_t i=0;i<tr.size();++i) printf("%zu %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n",i,tr.t[i],tr.E[i].x,tr.E[i].y,tr.E[i].z,r.S[i].x,r.S[i].y,r.S[i].z);
}
