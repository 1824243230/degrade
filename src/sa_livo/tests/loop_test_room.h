#pragma once
#include "loop_geometry.h"
inline sa_loop::Cloud::Ptr loopTestRoom() {
  sa_loop::Cloud::Ptr room(new sa_loop::Cloud);
  auto point=[&](double x,double y,double z){pcl::PointXYZI p;p.x=x+.017;p.y=y+.023;p.z=z+.031;p.intensity=1;room->push_back(p);};
  for(int x=-40;x<=40;++x)for(int y=-40;y<=40;++y)point(x*.1,y*.1,0);
  for(int y=-40;y<=40;++y)for(int z=0;z<=30;++z){point(-4,y*.1,z*.1);point(4,y*.1,z*.1);}
  for(int x=-40;x<=40;++x)for(int z=0;z<=30;++z)point(x*.1,4,z*.1);
  for(int k=0;k<8;++k)for(int u=0;u<=8;++u)for(int z=0;z<=20;++z){
    double x=-3+(k%4)*1.6,y=-2+(k/4)*3;
    point(x+u*.1,y,z*.1);point(x,y+u*.1,z*.1);
  }
  return room;
}
