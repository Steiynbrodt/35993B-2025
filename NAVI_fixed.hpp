#pragma once
#include "vex.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <utility>
#include <vector>
#include <map>

using namespace vex;

// ================= Config toggles =================
#ifndef NAVI_ENABLE_UI
#define NAVI_ENABLE_UI 0
#endif
#ifndef NAVI_ENABLE_LOGGER
#define NAVI_ENABLE_LOGGER 0
#endif
#ifndef NAVI_ENABLE_CACHE
#define NAVI_ENABLE_CACHE 1
#endif

// =============== Globals / Parameters ===============
static double timeTaken = 0.0;
static int    replans   = 0;
static int    stucks    = 0;
static bool   enableLearning = false;

static double initialHeadingOffset = 0.0;

static const double FIELD_SIZE_MM = 3657.6;   // 12ft in mm
static const int    GRID_SIZE     = 73;       // ~50mm cells
static const int    OFFSET        = GRID_SIZE / 2;
static const double CELL_SIZE     = FIELD_SIZE_MM / GRID_SIZE;

static double waypointTolerance   = 40.0;   // mm
static double deviationThreshold  = 80.0;   // mm
static double headingToleranceDeg = 12.0;   // deg
static bool   pathSmoothingEnabled = true;

// =============== Utility ===============
template<typename T>
static inline T clampv(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline double norm360(double a){ a=fmod(a,360.0); if(a<0)a+=360.0; return a; }
static inline double angDiff(double from,double to){ double d=to-from; while(d>180)d-=360; while(d<=-180)d+=360; return d; }
static inline double clampd(double v,double lo,double hi){ return std::max(lo,std::min(hi,v)); }
static inline int    clampi(int v,int lo,int hi){ return std::max(lo,std::min(hi,v)); }

// =============== Grid / State ===============
static int startX = 0, startY = 0;
static int goalX  = 0, goalY  = 0;

static bool walkable[GRID_SIZE][GRID_SIZE];
static bool isPath [GRID_SIZE][GRID_SIZE];

static std::vector<std::pair<int,int>> pathWaypoints;

double gridToMM(int g){ return g * CELL_SIZE; }
int    toGridCoord(double mmv){ return clampi((int)std::round(mmv / CELL_SIZE), -OFFSET, OFFSET); }

// =============== Node pool (no heap) ===============
struct Node { int x,y; int gCost,hCost; Node* parent; int fCost()const{return gCost+hCost;} };
static Node nodePool[GRID_SIZE][GRID_SIZE];

int heuristic(int x1,int y1,int x2,int y2){
  int dx=std::abs(x1-x2), dy=std::abs(y1-y2);
  return 10*(dx+dy) + (14-20)*std::min(dx,dy);
}

bool isLineClear(int x0,int y0,int x1,int y1){
  int dx=std::abs(x1-x0), dy=std::abs(y1-y0);
  int sx=(x0<x1)?1:-1, sy=(y0<y1)?1:-1; int err=dx-dy;
  while(true){
    if(!walkable[y0+OFFSET][x0+OFFSET]) return false;
    if(x0==x1 && y0==y1) break;
    int e2=2*err;
    if(e2>-dy){ err-=dy; x0+=sx; }
    if(e2< dx){ err+=dx; y0+=sy; }
  }
  return true;
}

void simplifyPath(std::vector<std::pair<int,int>>& wps){
  if(wps.size()<3) return;
  std::vector<std::pair<int,int>> out; out.reserve(wps.size());
  out.push_back(wps[0]);
  int dxPrev = wps[1].first - wps[0].first;
  int dyPrev = wps[1].second- wps[0].second;
  int len = std::max(std::abs(dxPrev), std::abs(dyPrev));
  if(len!=0){ dxPrev/=len; dyPrev/=len; }
  for(size_t i=1;i+1<wps.size();++i){
    int dx = wps[i+1].first - wps[i].first;
    int dy = wps[i+1].second- wps[i].second;
    int l  = std::max(std::abs(dx), std::abs(dy));
    if(l!=0){ dx/=l; dy/=l; }
    if(dx!=dxPrev || dy!=dyPrev){ out.push_back(wps[i]); dxPrev=dx; dyPrev=dy; }
  }
  out.push_back(wps.back());
  wps.swap(out);
}

void smoothPath(std::vector<std::pair<int,int>>& wps){
  if(wps.size()<3) return;
  std::vector<std::pair<int,int>> out; out.reserve(wps.size());
  size_t i=0;
  while(i<wps.size()){
    out.push_back(wps[i]);
    size_t j = wps.size()-1;
    for(; j>i+1; --j){
      if(isLineClear(wps[i].first,wps[i].second,wps[j].first,wps[j].second)) break;
    }
    i=j;
  }
  if(out.back()!=wps.back()) out.push_back(wps.back());
  wps.swap(out);
}

// =============== Heading fusion ===============
void calibrateINSFromGPS(){
  double gpsH = norm360(GPS17.heading(degrees));
  double insH = norm360(INS.heading (degrees));
  initialHeadingOffset = angDiff(insH, gpsH);
}
double getFusedHeading360(){
  // slow corrective blend
  double gpsH = norm360(GPS17.heading(degrees));
  double insH = norm360(INS.heading (degrees));
  double desiredOffset = angDiff(insH, gpsH);
  initialHeadingOffset = initialHeadingOffset*0.98 + desiredOffset*0.02;
  return norm360(insH + initialHeadingOffset);
}

// =============== Turn helper ===============
bool turnToHeadingAbs(double targetDeg,double stopTolDeg=1.5,int maxMs=1200){
  targetDeg = norm360(targetDeg);
  const double minPct=6.0, maxPct=40.0, deadband=1.0;
  int elapsed=0;
  while(elapsed<maxMs){
    double curr=getFusedHeading360();
    double err = angDiff(curr,targetDeg);
    if(std::fabs(err)<=std::max(deadband,stopTolDeg)) break;
    double pct = std::max(minPct, maxPct*(std::fabs(err)/90.0));
    if(std::fabs(err)<10.0) pct = std::max(4.0, pct*0.25);
    double sgn = (err>0)?+1.0:-1.0;
    LeftDrivetrain.spin (fwd,  pct*sgn, percent);
    RightDrivetrain.spin(fwd, -pct*sgn, percent);
    this_thread::sleep_for(10);
    elapsed+=10;
  }
  LeftDrivetrain.stop(); RightDrivetrain.stop();
  return true;
}

// =============== Logging (optional) ===============
void logGPSData(){
#if NAVI_ENABLE_LOGGER
  if(!Brain.SDcard.isInserted()) return;
  FILE* f=fopen("gps_log.txt","a"); if(!f) return;
  double x=GPS17.xPosition(mm), y=GPS17.yPosition(mm), h=getFusedHeading360();
  fprintf(f,"%.2f,%.2f,%.2f,%.2f\n", Brain.timer(sec), x,y,h);
  fclose(f);
#endif
}

// =============== Obstacles from Manual (approx) ===============
void addObstacleWithMargin(int gx,int gy){
  for(int dx=-1; dx<=1; ++dx)
    for(int dy=-1; dy<=1; ++dy){
      int nx=gx+dx, ny=gy+dy;
      if(nx>=-OFFSET && nx<=OFFSET && ny>=-OFFSET && ny<=OFFSET)
        walkable[ny+OFFSET][nx+OFFSET]=false;
    }
}
void addRectMM(double xMin,double yMin,double xMax,double yMax){
  int gx0=toGridCoord(xMin), gy0=toGridCoord(yMin);
  int gx1=toGridCoord(xMax), gy1=toGridCoord(yMax);
  for(int gy=std::min(gy0,gy1); gy<=std::max(gy0,gy1); ++gy)
    for(int gx=std::min(gx0,gx1); gx<=std::max(gx0,gx1); ++gx)
      if(gx>=-OFFSET && gx<=OFFSET && gy>=-OFFSET && gy<=OFFSET)
        walkable[gy+OFFSET][gx+OFFSET]=false;
}
void configureObstaclesFromManual(){
  for(int y=0;y<GRID_SIZE;++y) for(int x=0;x<GRID_SIZE;++x) walkable[y][x]=true;
  // Perimeter walls
  for(int i=-OFFSET; i<=OFFSET; ++i){
    addObstacleWithMargin(i, -OFFSET);
    addObstacleWithMargin(i,  OFFSET);
    addObstacleWithMargin(-OFFSET, i);
    addObstacleWithMargin( OFFSET, i);
  }
  // Long goals near north/south edges (~120mm depth no-go strip)
  const double inset = 120.0;
  addRectMM(-FIELD_SIZE_MM/2,  FIELD_SIZE_MM/2 - inset,  FIELD_SIZE_MM/2,  FIELD_SIZE_MM/2);
  addRectMM(-FIELD_SIZE_MM/2, -FIELD_SIZE_MM/2,          FIELD_SIZE_MM/2, -FIELD_SIZE_MM/2 + inset);
  // Center goals approx squares (~250mm)
  const double gOff = FIELD_SIZE_MM/6.0, gHalf=125.0;
  addRectMM(+gOff-gHalf, +gOff-gHalf, +gOff+gHalf, +gOff+gHalf);
  addRectMM(-gOff-gHalf, -gOff-gHalf, -gOff+gHalf, -gOff+gHalf);
  // Park zones per manual ~479x428mm corners
  const double pzw=479.0, pzd=428.0;
  addRectMM(-FIELD_SIZE_MM/2, -FIELD_SIZE_MM/2, -FIELD_SIZE_MM/2 + pzw, -FIELD_SIZE_MM/2 + pzd);
  addRectMM( FIELD_SIZE_MM/2 - pzw, FIELD_SIZE_MM/2 - pzd, FIELD_SIZE_MM/2, FIELD_SIZE_MM/2);
}

// =============== Start update ===============
void updateStartPositionFromGPS(){
  double gx=GPS17.xPosition(mm), gy=GPS17.yPosition(mm);
  if(std::isnan(gx)||std::isnan(gy)){ return; }
  startX=toGridCoord(gx); startY=toGridCoord(gy);
}

// =============== A* (no heap) ===============
void calculatePath(){
  for(int y=0;y<GRID_SIZE;++y)
    for(int x=0;x<GRID_SIZE;++x){
      isPath[y][x]=false;
      nodePool[y][x].x=x-OFFSET; nodePool[y][x].y=y-OFFSET;
      nodePool[y][x].gCost=999999; nodePool[y][x].hCost=999999; nodePool[y][x].parent=nullptr;
    }
  pathWaypoints.clear();
  Node* start=&nodePool[startY+OFFSET][startX+OFFSET];
  Node* goal =&nodePool[goalY +OFFSET][goalX +OFFSET];
  start->gCost=0; start->hCost=heuristic(startX,startY,goalX,goalY);

  std::vector<Node*> openSet; openSet.reserve(GRID_SIZE*GRID_SIZE/8);
  openSet.push_back(start);
  static bool closed[GRID_SIZE][GRID_SIZE];
  for(int y=0;y<GRID_SIZE;++y) for(int x=0;x<GRID_SIZE;++x) closed[y][x]=false;

  const int DX[8]={-1,-1,0,1,1,1,0,-1};
  const int DY[8]={ 0,-1,-1,-1,0,1,1, 1};

  bool found=false;
  while(!openSet.empty()){
    auto it=std::min_element(openSet.begin(),openSet.end(),
      [](Node* a,Node* b){ return a->fCost()<b->fCost(); });
    Node* cur=*it; openSet.erase(it);
    int ix=cur->x+OFFSET, iy=cur->y+OFFSET;
    closed[iy][ix]=true;
    if(cur==goal){
      found=true;
      Node* p=goal;
      while(p && p!=start){
        isPath[p->y+OFFSET][p->x+OFFSET]=true;
        pathWaypoints.emplace_back(p->x,p->y);
        p=p->parent;
      }
      std::reverse(pathWaypoints.begin(), pathWaypoints.end());
      simplifyPath(pathWaypoints);
      if(pathSmoothingEnabled) smoothPath(pathWaypoints);
      break;
    }
    for(int d=0; d<8; ++d){
      int nx=cur->x+DX[d], ny=cur->y+DY[d];
      if(nx<-OFFSET||nx>OFFSET||ny<-OFFSET||ny>OFFSET) continue;
      if(!walkable[ny+OFFSET][nx+OFFSET]) continue;
      if(closed[ny+OFFSET][nx+OFFSET]) continue;
      int stepCost=(DX[d]==0||DY[d]==0)?10:14;
      int tentative=cur->gCost+stepCost;
      Node* n=&nodePool[ny+OFFSET][nx+OFFSET];
      if(tentative < n->gCost){
        n->gCost=tentative;
        n->hCost=heuristic(nx,ny,goalX,goalY);
        n->parent=cur;
        openSet.push_back(n);
      }
    }
  }
  if(!found){
    pathWaypoints.clear();
  }
}

// =============== Drive helper ===============
enum class DriveResult { Arrived, Stalled };
DriveResult driveToWithRecovery(double targetXmm,double targetYmm){
  const double tolMM=40.0, basePct=10.0;
  double lastProgress=1e9; int stagnantMs=0;
  while(true){
    double cx=GPS17.xPosition(mm), cy=GPS17.yPosition(mm);
    double dx=targetXmm-cx, dy=targetYmm-cy;
    double dist=std::sqrt(dx*dx+dy*dy);
    if(dist<=tolMM){ FullDrivetrain.stop(); return DriveResult::Arrived; }
    double angle=std::atan2(dy,dx)*180.0/M_PI; if(angle<0) angle+=360.0;
    double currH=getFusedHeading360();
    double err=angDiff(currH,angle);
    bool reverse=std::fabs(err)>90.0;
    double fixed= reverse? norm360(angle+180.0): angle;
    turnToHeadingAbs(fixed,1.5,800);
    double pct= reverse? -basePct : basePct;
    if(dist<200)pct*=0.6; if(dist<100)pct*=0.4;
    LeftDrivetrain.spin (fwd,pct,percent);
    RightDrivetrain.spin(fwd,pct,percent);
    if(dist < lastProgress - 8.0){ lastProgress=dist; stagnantMs=0; }
    else stagnantMs+=50;
    if(stagnantMs>1000){
      double aheadX=cx+150.0*std::cos(angle*M_PI/180.0);
      double aheadY=cy+150.0*std::sin(angle*M_PI/180.0);
      addObstacleWithMargin(toGridCoord(aheadX), toGridCoord(aheadY));
      FullDrivetrain.stop();
      return DriveResult::Stalled;
    }
    wait(50,msec);
  }
}

// =============== Follow path ===============
void followPath(double t0,double timeBudgetSec){
  if(pathWaypoints.empty()) return;
  bool reached=false; size_t i=0;
  while(!reached && i<pathWaypoints.size()){
    if(Brain.timer(sec)-t0 > timeBudgetSec){ FullDrivetrain.stop(); return; }
    while(i+1<pathWaypoints.size()){
      auto [ax,ay]=pathWaypoints[i]; auto [bx,by]=pathWaypoints[i+1];
      if(isLineClear(ax,ay,bx,by)) i++; else break;
    }
    auto [gx,gy]=pathWaypoints[i];
    double tx=gridToMM(gx), ty=gridToMM(gy);
    auto r=driveToWithRecovery(tx,ty);
    if(r==DriveResult::Stalled){
      updateStartPositionFromGPS();
      calculatePath(); replans++; i=0; continue;
    }
    ++i; if(i>=pathWaypoints.size()) reached=true;
  }
}

// =============== Cache ===============
#if NAVI_ENABLE_CACHE
struct Key{ int sx,sy,gx,gy; };
struct KeyLess{
  bool operator()(const Key& a,const Key& b) const{
    if(a.sx!=b.sx) return a.sx<b.sx;
    if(a.sy!=b.sy) return a.sy<b.sy;
    if(a.gx!=b.gx) return a.gx<b.gx;
    return a.gy<b.gy;
  }
};
static std::map<Key,std::vector<std::pair<int,int>>,KeyLess> g_pathCache;
#endif

// =============== NAVI entry ===============
void NAVI(double targetXmm,double targetYmm,double timeBudgetSec=1e9){
  double t0=Brain.timer(sec);
  configureObstaclesFromManual();
  goalX=clampi(toGridCoord(targetXmm), -OFFSET, OFFSET);
  goalY=clampi(toGridCoord(targetYmm), -OFFSET, OFFSET);
  updateStartPositionFromGPS();
#if NAVI_ENABLE_CACHE
  Key k{startX,startY,goalX,goalY};
  auto it=g_pathCache.find(k);
  if(it!=g_pathCache.end()){
    pathWaypoints=it->second;
    followPath(t0,timeBudgetSec);
    return;
  }
#endif
  calculatePath();
  followPath(t0,timeBudgetSec);
#if NAVI_ENABLE_CACHE
  if(!pathWaypoints.empty()) g_pathCache[k]=pathWaypoints;
#endif
  timeTaken=Brain.timer(sec)-t0;
}
