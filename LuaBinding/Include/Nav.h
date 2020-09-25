#pragma once
#include <stdint.h>
#include <vector>
#include <map>
#include "Recast.h"
#include "InputGeom.h"
#include "SampleInterfaces.h"
#include "DetourNavMeshQuery.h"
#include "time.h"
/*
enum SamplePolyAreas
{
	SAMPLE_POLYAREA_GROUND,
	SAMPLE_POLYAREA_WATER,
	SAMPLE_POLYAREA_ROAD,
	SAMPLE_POLYAREA_DOOR,
	SAMPLE_POLYAREA_GRASS,
	SAMPLE_POLYAREA_JUMP,
};

enum SamplePolyFlags
{
	SAMPLE_POLYFLAGS_WALK		= 0x01,
	SAMPLE_POLYFLAGS_SWIM		= 0x02,
	SAMPLE_POLYFLAGS_DOOR		= 0x04,
	SAMPLE_POLYFLAGS_JUMP		= 0x08,
	SAMPLE_POLYFLAGS_DISABLED	= 0x10,
	SAMPLE_POLYFLAGS_ALL		= 0xffff
};

enum SamplePartitionType
{
	SAMPLE_PARTITION_WATERSHED,
	SAMPLE_PARTITION_MONOTONE,
	SAMPLE_PARTITION_LAYERS,
};*/

class Nav{
protected:
	class InputGeom* m_geom;
	class dtNavMesh* m_navMesh;
	class dtNavMeshQuery* m_navQuery;
	class dtCrowd* m_crowd;
	BuildContext* m_ctx;

	float m_cellSize;
	float m_cellHeight;
	float m_agentHeight;
	float m_agentRadius;
	float m_agentMaxClimb;
	float m_agentMaxSlope;
	float m_regionMinSize;
	float m_regionMergeSize;
	float m_edgeMaxLen;
	float m_edgeMaxError;
	float m_vertsPerPoly;
	float m_detailSampleDist;
	float m_detailSampleMaxError;
	int m_partitionType;

	bool m_filterLowHangingObstacles;
	bool m_filterLedgeSpans;
	bool m_filterWalkableLowHeightSpans;

	//--solomesh
	static const int MAX_POLYS = 256;
	static const int MAX_SMOOTH = 2048;
	bool m_keepInterResults;
	float m_totalBuildTimeMs;

	unsigned char* m_triareas;
	rcHeightfield* m_solid;
	rcCompactHeightfield* m_chf;
	rcContourSet* m_cset;
	rcPolyMesh* m_pmesh;
	rcConfig m_cfg;	
	rcPolyMeshDetail* m_dmesh;
	dtQueryFilter m_filter;
	float m_polyPickExt[3];
	//--end solomesh

	void cleanup();
public:
	Nav();
	virtual ~Nav();

	void resetCommonSettings();
	void setContext(BuildContext* ctx) { m_ctx = ctx; }
	void setSetting(float cellSize,float cellHeight,float agentHeight,float agentRadius,float agentMaxClimb,float agentMaxSlope);
	class dtNavMesh* getNavMesh() { return m_navMesh; }
	class dtNavMeshQuery* getNavMeshQuery() { return m_navQuery; }
	float getAgentRadius() { return m_agentRadius; }
	float getAgentHeight() { return m_agentHeight; }
	float getAgentClimb() { return m_agentMaxClimb; }
	float getTotalBuildTimeMs(){return m_totalBuildTimeMs;}
	dtStatus findNearestPoly(const float* pos,dtPolyRef* ref,float* nearestPos);
	bool findSmoothPath(const float* spos,const float* epos,float* path,int* npath);
	bool findStraightPath(const float* spos,const float* epos,float* path,int* npath);
	bool load(const std::string& filepath);
	bool handleBuild();
	bool getHeight(const float* pos,float* h);
	bool getHeightByHit(const float* pos,const float diff,float* h);
	bool getHitPos(const float* spos,const float* epos,float* pos);
	bool findRandomPos(float* pos);
	//bool findPath(const float* spos,float* path,int* npath);
	bool findPath(unsigned int pathFindType,const float* spos,const float* epos,float* path,int* npath);
	bool getNearestPos(const float* pos, float* nearestPos);
	bool isWalkable(const float* pos,const float maxDist, float* h,float* dist);
	float getClock();
};

class Crowd{
	Nav* m_nav;
	dtCrowd* m_crowd;

	float m_targetPos[3];
	dtPolyRef m_targetRef;

	static const int AGENT_MAX_TRAIL = 64;
	static const int MAX_AGENTS = 128;

public:
	Crowd();
	virtual ~Crowd();

	void init(Nav* nav);
	int addAgent(const float* pos);
	void removeAgent(const int idx);
	bool setMoveTarget(int idx,const float* p, bool adjust,float* targetPos);
	void updateTick(const float dt);
	void getAgentPos(const int idx,float* pos,int* state,int* targetState);
};