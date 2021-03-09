#pragma warning( push )
#pragma warning( disable : 4244)

#include <string.h>
#include <stdio.h>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include "Nav.h"
#include "DetourNavMeshQuery.h"
#include "DetourNavMeshBuilder.h"
#include "DetourCrowd.h"
#include "DetourCommon.h"
#include "Sample.h"

using std::string;
using namespace std;

static float frand()
{
	return (float)rand()/(float)RAND_MAX;
}

inline bool inRange(const float* v1, const float* v2, const float r, const float h)
{
	const float dx = v2[0] - v1[0];
	const float dy = v2[1] - v1[1];
	const float dz = v2[2] - v1[2];
	return (dx*dx + dz*dz) < r*r && fabsf(dy) < h;
}

static int fixupCorridor(dtPolyRef* path, const int npath, const int maxPath,
						 const dtPolyRef* visited, const int nvisited)
{
	int furthestPath = -1;
	int furthestVisited = -1;
	
	// Find furthest common polygon.
	for (int i = npath-1; i >= 0; --i)
	{
		bool found = false;
		for (int j = nvisited-1; j >= 0; --j)
		{
			if (path[i] == visited[j])
			{
				furthestPath = i;
				furthestVisited = j;
				found = true;
			}
		}
		if (found)
			break;
	}

	// If no intersection found just return current path. 
	if (furthestPath == -1 || furthestVisited == -1)
		return npath;
	
	// Concatenate paths.	

	// Adjust beginning of the buffer to include the visited.
	const int req = nvisited - furthestVisited;
	const int orig = rcMin(furthestPath+1, npath);
	int size = rcMax(0, npath-orig);
	if (req+size > maxPath)
		size = maxPath-req;
	if (size)
		memmove(path+req, path+orig, size*sizeof(dtPolyRef));
	
	// Store visited
	for (int i = 0; i < req; ++i)
		path[i] = visited[(nvisited-1)-i];				
	
	return req+size;
}

// This function checks if the path has a small U-turn, that is,
// a polygon further in the path is adjacent to the first polygon
// in the path. If that happens, a shortcut is taken.
// This can happen if the target (T) location is at tile boundary,
// and we're (S) approaching it parallel to the tile edge.
// The choice at the vertex can be arbitrary, 
//  +---+---+
//  |:::|:::|
//  +-S-+-T-+
//  |:::|   | <-- the step can end up in here, resulting U-turn path.
//  +---+---+
static int fixupShortcuts(dtPolyRef* path, int npath, dtNavMeshQuery* navQuery)
{
	if (npath < 3)
		return npath;

	// Get connected polygons
	static const int maxNeis = 16;
	dtPolyRef neis[maxNeis];
	int nneis = 0;

	const dtMeshTile* tile = 0;
	const dtPoly* poly = 0;
	if (dtStatusFailed(navQuery->getAttachedNavMesh()->getTileAndPolyByRef(path[0], &tile, &poly)))
		return npath;
	
	for (unsigned int k = poly->firstLink; k != DT_NULL_LINK; k = tile->links[k].next)
	{
		const dtLink* link = &tile->links[k];
		if (link->ref != 0)
		{
			if (nneis < maxNeis)
				neis[nneis++] = link->ref;
		}
	}

	// If any of the neighbour polygons is within the next few polygons
	// in the path, short cut to that polygon directly.
	static const int maxLookAhead = 6;
	int cut = 0;
	for (int i = dtMin(maxLookAhead, npath) - 1; i > 1 && cut == 0; i--) {
		for (int j = 0; j < nneis; j++)
		{
			if (path[i] == neis[j]) {
				cut = i;
				break;
			}
		}
	}
	if (cut > 1)
	{
		int offset = cut-1;
		npath -= offset;
		for (int i = 1; i < npath; i++)
			path[i] = path[i+offset];
	}

	return npath;
}

static bool getSteerTarget(dtNavMeshQuery* navQuery, const float* startPos, const float* endPos,
						   const float minTargetDist,
						   const dtPolyRef* path, const int pathSize,
						   float* steerPos, unsigned char& steerPosFlag, dtPolyRef& steerPosRef,
						   float* outPoints = 0, int* outPointCount = 0)							 
{
	// Find steer target.
	static const int MAX_STEER_POINTS = 3;
	float steerPath[MAX_STEER_POINTS*3];
	unsigned char steerPathFlags[MAX_STEER_POINTS];
	dtPolyRef steerPathPolys[MAX_STEER_POINTS];
	int nsteerPath = 0;
	navQuery->findStraightPath(startPos, endPos, path, pathSize,
							   steerPath, steerPathFlags, steerPathPolys, &nsteerPath, MAX_STEER_POINTS);
	if (!nsteerPath)
		return false;
		
	if (outPoints && outPointCount)
	{
		*outPointCount = nsteerPath;
		for (int i = 0; i < nsteerPath; ++i)
			dtVcopy(&outPoints[i*3], &steerPath[i*3]);
	}

	// Find vertex far enough to steer to.
	int ns = 0;
	while (ns < nsteerPath)
	{
		// Stop at Off-Mesh link or when point is further than slop away.
		if ((steerPathFlags[ns] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ||
			!inRange(&steerPath[ns*3], startPos, minTargetDist, 1000.0f))
			break;
		ns++;
	}
	// Failed to find good point to steer to.
	if (ns >= nsteerPath)
		return false;
	
	dtVcopy(steerPos, &steerPath[ns*3]);
	steerPos[1] = startPos[1];
	steerPosFlag = steerPathFlags[ns];
	steerPosRef = steerPathPolys[ns];
	
	return true;
}

Nav::Nav():
	m_navMesh(0),
	m_navQuery(0),
	m_crowd(0),
	m_ctx(0),
	m_filterLowHangingObstacles(true),
	m_filterLedgeSpans(true),
	m_filterWalkableLowHeightSpans(true),
	m_keepInterResults(true),
	m_totalBuildTimeMs(0),
	m_triareas(0),
	m_solid(0),
	m_chf(0),
	m_cset(0),
	m_pmesh(0),
	m_dmesh(0)
{
	resetCommonSettings();
	m_navQuery = dtAllocNavMeshQuery();
	m_crowd = dtAllocCrowd();

	m_filter.setIncludeFlags(SAMPLE_POLYFLAGS_ALL ^ SAMPLE_POLYFLAGS_DISABLED);
	m_filter.setExcludeFlags(0);

	if (m_navQuery)
	{
		// Change costs.
		m_filter.setAreaCost(SAMPLE_POLYAREA_GROUND, 1.0f);
		m_filter.setAreaCost(SAMPLE_POLYAREA_WATER, 10.0f);
		m_filter.setAreaCost(SAMPLE_POLYAREA_ROAD, 1.0f);
		m_filter.setAreaCost(SAMPLE_POLYAREA_DOOR, 1.0f);
		m_filter.setAreaCost(SAMPLE_POLYAREA_GRASS, 2.0f);
		m_filter.setAreaCost(SAMPLE_POLYAREA_JUMP, 1.5f);
	}

	m_polyPickExt[0] = 2;
	m_polyPickExt[1] = 4;
	m_polyPickExt[2] = 2;

	m_geom = new InputGeom;
}

Nav::~Nav()
{
	dtFreeNavMeshQuery(m_navQuery);
	dtFreeCrowd(m_crowd);

	delete m_geom;

	cleanup();
}

void Nav::cleanup()
{
	delete [] m_triareas;
	m_triareas = 0;
	rcFreeHeightField(m_solid);
	m_solid = 0;
	rcFreeCompactHeightfield(m_chf);
	m_chf = 0;
	rcFreeContourSet(m_cset);
	m_cset = 0;
	rcFreePolyMesh(m_pmesh);
	m_pmesh = 0;
	rcFreePolyMeshDetail(m_dmesh);
	m_dmesh = 0;
	dtFreeNavMesh(m_navMesh);
	m_navMesh = 0;
}

void Nav::resetCommonSettings()
{
	m_cellSize = 0.3f;
	m_cellHeight = 0.2f;
	m_agentHeight = 2.0f;
	m_agentRadius = 0.6f;
	m_agentMaxClimb = 0.6f;
	m_agentMaxSlope = 45.0f;
	m_regionMinSize = 2;
	m_regionMergeSize = 20;
	m_edgeMaxLen = 12.0f;
	m_edgeMaxError = 1.3f;
	m_vertsPerPoly = 6.0f;
	m_detailSampleDist = 6.0f;
	m_detailSampleMaxError = 1.0f;
	m_partitionType = SAMPLE_PARTITION_WATERSHED;
}

void Nav::setSetting(float cellSize,float cellHeight,float agentHeight,float agentRadius,float agentMaxClimb,float agentMaxSlope){
	m_cellSize = cellSize;
	m_cellHeight = cellHeight;
	m_agentHeight = agentHeight;
	m_agentRadius = agentRadius;
	m_agentMaxClimb = agentMaxClimb;
	m_agentMaxSlope = agentMaxSlope;
}

bool Nav::load(const std::string& filepath)
{
	return m_geom->load(m_ctx,filepath);
}

float Nav::getClock(){
	return ((double)clock())/(double)CLOCKS_PER_SEC;
}

bool Nav::handleBuild()
{
	float startTime = getClock();
	//printf("build start:%.2f\n",startTime);
	if (!m_geom || !m_geom->getMesh())
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Input mesh is not specified.");
		return false;
	}
	m_ctx->log(RC_LOG_PROGRESS, "Sample_SoloMesh::handleBuild start:");
	cleanup();

	const float* bmin = m_geom->getNavMeshBoundsMin();
	const float* bmax = m_geom->getNavMeshBoundsMax();
	const float* verts = m_geom->getMesh()->getVerts();
	const int nverts = m_geom->getMesh()->getVertCount();
	const int* tris = m_geom->getMesh()->getTris();
	const int ntris = m_geom->getMesh()->getTriCount();
	
	// Step 1. Initialize build config.
	float time0 = getClock();
	
	memset(&m_cfg, 0, sizeof(m_cfg));
	m_cfg.cs = m_cellSize;
	m_cfg.ch = m_cellHeight;
	m_cfg.walkableSlopeAngle = m_agentMaxSlope;
	m_cfg.walkableHeight = (int)ceilf(m_agentHeight / m_cfg.ch);
	m_cfg.walkableClimb = (int)floorf(m_agentMaxClimb / m_cfg.ch);
	m_cfg.walkableRadius = (int)ceilf(m_agentRadius / m_cfg.cs);
	m_cfg.maxEdgeLen = (int)(m_edgeMaxLen / m_cellSize);
	m_cfg.maxSimplificationError = m_edgeMaxError;
	m_cfg.minRegionArea = (int)rcSqr(m_regionMinSize);		// Note: area = size*size
	m_cfg.mergeRegionArea = (int)rcSqr(m_regionMergeSize);	// Note: area = size*size
	m_cfg.maxVertsPerPoly = (int)m_vertsPerPoly;
	m_cfg.detailSampleDist = m_detailSampleDist < 0.9f ? 0 : m_cellSize * m_detailSampleDist;
	m_cfg.detailSampleMaxError = m_cellHeight * m_detailSampleMaxError;
	
	rcVcopy(m_cfg.bmin, bmin);
	rcVcopy(m_cfg.bmax, bmax);
	rcCalcGridSize(m_cfg.bmin, m_cfg.bmax, m_cfg.cs, &m_cfg.width, &m_cfg.height);

	// Reset build times gathering.
	m_ctx->resetTimers();

	// Start the build process.	
	m_ctx->startTimer(RC_TIMER_TOTAL);
	
	m_ctx->log(RC_LOG_PROGRESS, "Building navigation:");
	m_ctx->log(RC_LOG_PROGRESS, " - %d x %d cells", m_cfg.width, m_cfg.height);
	m_ctx->log(RC_LOG_PROGRESS, " - %.1fK verts, %.1fK tris", nverts/1000.0f, ntris/1000.0f);

	float time1 = getClock();
	//printf("build time1:%.2f\n",time1-time0);

	// Step 2. Rasterize input polygon soup.
	m_solid = rcAllocHeightfield();
	if (!m_solid)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
		return false;
	}
	if (!rcCreateHeightfield(m_ctx, *m_solid, m_cfg.width, m_cfg.height, m_cfg.bmin, m_cfg.bmax, m_cfg.cs, m_cfg.ch))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
		return false;
	}

	m_triareas = new unsigned char[ntris];
	if (!m_triareas)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_triareas' (%d).", ntris);
		return false;
	}
	
	memset(m_triareas, 0, ntris*sizeof(unsigned char));
	rcMarkWalkableTriangles(m_ctx, m_cfg.walkableSlopeAngle, verts, nverts, tris, ntris, m_triareas);
	if (!rcRasterizeTriangles(m_ctx, verts, nverts, tris, m_triareas, ntris, *m_solid, m_cfg.walkableClimb))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not rasterize triangles.");
		return false;
	}

	if (!m_keepInterResults)
	{
		delete [] m_triareas;
		m_triareas = 0;
	}
	
	float time2 = getClock();
	//printf("build time2:%.2f\n",time2-time1);

	// Step 3. Filter walkables surfaces.
	if (m_filterLowHangingObstacles)
		rcFilterLowHangingWalkableObstacles(m_ctx, m_cfg.walkableClimb, *m_solid);
	if (m_filterLedgeSpans)
		rcFilterLedgeSpans(m_ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, *m_solid);
	if (m_filterWalkableLowHeightSpans)
		rcFilterWalkableLowHeightSpans(m_ctx, m_cfg.walkableHeight, *m_solid);

	float time3 = getClock();
	//printf("build time3:%.2f\n",time3-time2);

	// Step 4. Partition walkable surface to simple regions.
	m_chf = rcAllocCompactHeightfield();
	if (!m_chf)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
		return false;
	}
	if (!rcBuildCompactHeightfield(m_ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, *m_solid, *m_chf))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
		return false;
	}
	
	if (!m_keepInterResults)
	{
		rcFreeHeightField(m_solid);
		m_solid = 0;
	}
		
	// Erode the walkable area by agent radius.
	if (!rcErodeWalkableArea(m_ctx, m_cfg.walkableRadius, *m_chf))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
		return false;
	}

	float time4 = getClock();
	//printf("build time4:%.2f\n",time4-time3);

	// (Optional) Mark areas.
	const ConvexVolume* vols = m_geom->getConvexVolumes();
	for (int i  = 0; i < m_geom->getConvexVolumeCount(); ++i)
		rcMarkConvexPolyArea(m_ctx, vols[i].verts, vols[i].nverts, vols[i].hmin, vols[i].hmax, (unsigned char)vols[i].area, *m_chf);

	float time4_1 = getClock();
	//printf("build time4_1:%.2f\n",time4_1-time4);

	if (m_partitionType == SAMPLE_PARTITION_WATERSHED)
	{
		
		if (!rcBuildDistanceField(m_ctx, *m_chf))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build distance field.");
			return false;
		}

		if (!rcBuildRegions(m_ctx, *m_chf, 0, m_cfg.minRegionArea, m_cfg.mergeRegionArea))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build watershed regions.");
			return false;
		}
		float time4_2 = getClock();
		//printf("build time4_2:%.2f\n",time4_2-time4_1);
	}
	else if (m_partitionType == SAMPLE_PARTITION_MONOTONE)
	{
		if (!rcBuildRegionsMonotone(m_ctx, *m_chf, 0, m_cfg.minRegionArea, m_cfg.mergeRegionArea))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build monotone regions.");
			return false;
		}
		float time4_3 = getClock();
		//printf("build time4_3:%.2f\n",time4_3-time4_1);
	}
	else
	{
		if (!rcBuildLayerRegions(m_ctx, *m_chf, 0, m_cfg.minRegionArea))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build layer regions.");
			return false;
		}
		float time4_4 = getClock();
		//printf("build time4_4:%.2f\n",time4_4-time4_1);
	}
	
	float time5 = getClock();
	//printf("build time5:%.2f\n",time5-time4);

	m_cset = rcAllocContourSet();
	if (!m_cset)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'.");
		return false;
	}
	if (!rcBuildContours(m_ctx, *m_chf, m_cfg.maxSimplificationError, m_cfg.maxEdgeLen, *m_cset))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create contours.");
		return false;
	}
	
	m_pmesh = rcAllocPolyMesh();
	if (!m_pmesh)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmesh'.");
		return false;
	}
	if (!rcBuildPolyMesh(m_ctx, *m_cset, m_cfg.maxVertsPerPoly, *m_pmesh))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not triangulate contours.");
		return false;
	}
	
	float time6 = getClock();
	//printf("build time6:%.2f\n",time6-time5);

	m_dmesh = rcAllocPolyMeshDetail();
	if (!m_dmesh)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmdtl'.");
		return false;
	}

	if (!rcBuildPolyMeshDetail(m_ctx, *m_pmesh, *m_chf, m_cfg.detailSampleDist, m_cfg.detailSampleMaxError, *m_dmesh))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build detail mesh.");
		return false;
	}

	float time7 = getClock();
	//printf("build time7:%.2f\n",time7-time6);

	if (!m_keepInterResults)
	{
		rcFreeCompactHeightfield(m_chf);
		m_chf = 0;
		rcFreeContourSet(m_cset);
		m_cset = 0;
	}

	if (m_cfg.maxVertsPerPoly <= DT_VERTS_PER_POLYGON)
	{
		unsigned char* navData = 0;
		int navDataSize = 0;

		for (int i = 0; i < m_pmesh->npolys; ++i)
		{
			if (m_pmesh->areas[i] == RC_WALKABLE_AREA)
				m_pmesh->areas[i] = SAMPLE_POLYAREA_GROUND;
				
			if (m_pmesh->areas[i] == SAMPLE_POLYAREA_GROUND ||
				m_pmesh->areas[i] == SAMPLE_POLYAREA_GRASS ||
				m_pmesh->areas[i] == SAMPLE_POLYAREA_ROAD)
			{
				m_pmesh->flags[i] = SAMPLE_POLYFLAGS_WALK;
			}
			else if (m_pmesh->areas[i] == SAMPLE_POLYAREA_WATER)
			{
				m_pmesh->flags[i] = SAMPLE_POLYFLAGS_SWIM;
			}
			else if (m_pmesh->areas[i] >= SAMPLE_POLYAREA_DOOR)
			{
				//define 5~63 areas as doors
				//each 0xf doors has one query flag
				//4 flags total
				unsigned char area = m_pmesh->areas[i];
				m_pmesh->areas[i] = area;
				unsigned short flag = pow(2, (area - SAMPLE_POLYAREA_DOOR) & 0xf);
			//	m_pmesh->flags[i] = SAMPLE_POLYFLAGS_WALK | flag;
				m_pmesh->flags[i] = flag;
			}
		}

		dtNavMeshCreateParams params;
		memset(&params, 0, sizeof(params));
		params.verts = m_pmesh->verts;
		params.vertCount = m_pmesh->nverts;
		params.polys = m_pmesh->polys;
		params.polyAreas = m_pmesh->areas;
		params.polyFlags = m_pmesh->flags;
		params.polyCount = m_pmesh->npolys;
		params.nvp = m_pmesh->nvp;
		params.detailMeshes = m_dmesh->meshes;
		params.detailVerts = m_dmesh->verts;
		params.detailVertsCount = m_dmesh->nverts;
		params.detailTris = m_dmesh->tris;
		params.detailTriCount = m_dmesh->ntris;
		params.offMeshConVerts = m_geom->getOffMeshConnectionVerts();
		params.offMeshConRad = m_geom->getOffMeshConnectionRads();
		params.offMeshConDir = m_geom->getOffMeshConnectionDirs();
		params.offMeshConAreas = m_geom->getOffMeshConnectionAreas();
		params.offMeshConFlags = m_geom->getOffMeshConnectionFlags();
		params.offMeshConUserID = m_geom->getOffMeshConnectionId();
		params.offMeshConCount = m_geom->getOffMeshConnectionCount();
		params.walkableHeight = m_agentHeight;
		params.walkableRadius = m_agentRadius;
		params.walkableClimb = m_agentMaxClimb;
		rcVcopy(params.bmin, m_pmesh->bmin);
		rcVcopy(params.bmax, m_pmesh->bmax);
		params.cs = m_cfg.cs;
		params.ch = m_cfg.ch;
		params.buildBvTree = true;
		
		if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
		{
			m_ctx->log(RC_LOG_ERROR, "Could not build Detour navmesh.");
			return false;
		}
		
		float time8 = getClock();
		//printf("build time8:%.2f\n",time8-time7);

		m_navMesh = dtAllocNavMesh();
		if (!m_navMesh)
		{
			dtFree(navData);
			m_ctx->log(RC_LOG_ERROR, "Could not create Detour navmesh");
			return false;
		}
		
		dtStatus status;
		
		status = m_navMesh->init(navData, navDataSize, DT_TILE_FREE_DATA);
		if (dtStatusFailed(status))
		{
			dtFree(navData);
			m_ctx->log(RC_LOG_ERROR, "Could not init Detour navmesh");
			return false;
		}
		
		status = m_navQuery->init(m_navMesh, 2048);
		if (dtStatusFailed(status))
		{
			m_ctx->log(RC_LOG_ERROR, "Could not init Detour navmesh query");
			return false;
		}

		float time9 = getClock();
		//printf("build time9:%.2f\n",time9-time8);

	}
	
	m_ctx->stopTimer(RC_TIMER_TOTAL);

	duLogBuildTimes(*m_ctx, m_ctx->getAccumulatedTime(RC_TIMER_TOTAL));
	m_ctx->log(RC_LOG_PROGRESS, ">> Polymesh: %d vertices  %d polygons", m_pmesh->nverts, m_pmesh->npolys);
	
	m_totalBuildTimeMs = m_ctx->getAccumulatedTime(RC_TIMER_TOTAL)/1000.0f;
	TimeVal endTime = getClock();
	//printf("build   END:%.2f,%.2f\n",endTime,endTime-startTime);
	return true;
}

bool Nav::findRandomPos(float* pos){
	dtPolyRef m_startRef;
	dtStatus status = m_navQuery->findRandomPoint(&m_filter,frand,&m_startRef,pos);
	if(!dtStatusSucceed(status)){
		m_ctx->log(RC_LOG_ERROR, "Nav::findRandomPos ERROR:%d", status);
		return false;
	}
	return true;
}

bool Nav::getHeight(const float* pos,float* h){
	float nearestPos[3];
	bool bFound = false;
	dtPolyRef nearestRef = 0;
	dtStatus status = m_navQuery->findNearestPoly(pos, m_polyPickExt, &m_filter, &nearestRef, nearestPos);
	if(m_navQuery->isValidPolyRef(nearestRef,&m_filter))
	{
		status = m_navQuery->getPolyHeight(nearestRef,pos,h);
		if(dtStatusSucceed(status)){
			bFound = true;
		}
	}
	return bFound;
}

bool Nav::getHeightByHit(const float* pos,float diff,float* h){
	float hitTime=0;
	float spos[3];
	spos[0] = pos[0];
	spos[1] = pos[1]+diff;
	spos[2] = pos[2];
	float epos[3];
	epos[0] = pos[0];
	epos[1] = pos[1]-diff;
	epos[2] = pos[2];
	float flag = m_geom->raycastMesh(spos,epos,hitTime);
	if(flag){
		*h = spos[1] + (epos[1] - spos[1]) * hitTime;
		return true;
	}else{
		return false;
	}
}

bool Nav::getHitPos(const float* spos,const float* epos,float* pos){
	float hitTime=0;
	float flag = m_geom->raycastMesh((float*)spos,(float*)epos,hitTime);
	if(flag){
		pos[0] = spos[0] + (epos[0] - spos[0]) * hitTime;
		pos[1] = spos[1] + (epos[1] - spos[1]) * hitTime;
		pos[2] = spos[2] + (epos[2] - spos[2]) * hitTime;
		return true;
	}else{
		pos[0] = epos[0];
		pos[1] = epos[1];
		pos[2] = epos[2];
		return false;
	}
}

/*bool Nav::findPath(const float* spos,float* path,int* npath){
	float epos[3];
	dtStatus status;
	dtPolyRef m_endRef;
	status = m_navQuery->findRandomPoint(&m_filter,frand,&m_endRef,epos);
	if(!dtStatusSucceed(status)){
		m_ctx->log(RC_LOG_ERROR, "Pathfinding::find : Random epos ERROR:%d", status);
		return false;
	}
	return findPath(1,spos,epos,path,npath);
}*/

bool Nav::findPath(unsigned int pathFindType,const float* spos,const float* epos,float* path,int* npath,int* offmesh, int* noffmesh){
	if(pathFindType == 1){
		return findSmoothPath(spos,epos,path,npath,offmesh,noffmesh);
	}
	else if(pathFindType == 2)
	{
		return findStraightPath(spos,epos,path,npath);
	}
	return false;
}

bool Nav::findStraightPath(const float* spos,const float* epos,float* path,int* npath){
	int m_npolys=0;
	dtPolyRef m_polys[MAX_POLYS];
	dtPolyRef m_startRef,m_endRef;
	unsigned char m_straightPathFlags[MAX_POLYS];
	dtPolyRef m_straightPathPolys[MAX_POLYS];
	
	m_navQuery->findNearestPoly(spos, m_polyPickExt, &m_filter, &m_startRef, 0);
	m_navQuery->findNearestPoly(epos, m_polyPickExt, &m_filter, &m_endRef, 0);
	dtStatus status = m_navQuery->findPath(m_startRef, m_endRef, spos, epos, &m_filter, m_polys, &m_npolys, MAX_POLYS);
	if(dtStatusSucceed(status)){
		int m_straightPathOptions = DT_STRAIGHTPATH_ALL_CROSSINGS;
		*npath = 0;
		if(m_npolys){
			float nepos[3];
			dtVcopy(nepos,epos);
			if(m_polys[m_npolys-1] != m_endRef){
				m_navQuery->closestPointOnPoly(m_polys[m_npolys-1],epos,nepos,0);
			}
			m_navQuery->findStraightPath(spos,nepos,m_polys,m_npolys,
				path,m_straightPathFlags,
				m_straightPathPolys,npath,MAX_POLYS,m_straightPathOptions);
			return true;
		}
	}
	return false;
}

bool Nav::findSmoothPath(const float* spos,const float* epos,float* path,int* npath, int* offmesh, int* noffmesh)
{
	int m_npolys=0;
	*noffmesh = 0;
	dtPolyRef m_polys[MAX_POLYS];
	dtPolyRef m_startRef,m_endRef;
	
	m_navQuery->findNearestPoly(spos, m_polyPickExt, &m_filter, &m_startRef, 0);
	m_navQuery->findNearestPoly(epos, m_polyPickExt, &m_filter, &m_endRef, 0);
	dtStatus status = m_navQuery->findPath(m_startRef, m_endRef, spos, epos, &m_filter, m_polys, &m_npolys, MAX_POLYS);
	int m_pathIterNum = 0;
	if(dtStatusSucceed(status))
	{
		int m_nsmoothPath = 0;
		if (m_npolys)
		{
			// Iterate over the path to find smooth path on the detail mesh surface.
			dtPolyRef polys[MAX_POLYS];
			memcpy(polys, m_polys, sizeof(dtPolyRef)*m_npolys); 
			int npolys = m_npolys;
				
			float iterPos[3], targetPos[3];
			m_navQuery->closestPointOnPoly(m_startRef, spos, iterPos, 0);
			m_navQuery->closestPointOnPoly(polys[npolys-1], epos, targetPos, 0);
				
			static const float STEP_SIZE = 0.5f;
			static const float SLOP = 0.01f;
				
			m_nsmoothPath = 0;
				
			dtVcopy(&path[m_nsmoothPath*3], iterPos);
			m_nsmoothPath++;
				
			// Move towards target a small advancement at a time until target reached or
			// when ran out of memory to store the path.
			while (npolys && m_nsmoothPath < MAX_SMOOTH)
			{
				// Find location to steer towards.
				float steerPos[3];
				unsigned char steerPosFlag;
				dtPolyRef steerPosRef;
					
				if (!getSteerTarget(m_navQuery, iterPos, targetPos, SLOP,
									polys, npolys, steerPos, steerPosFlag, steerPosRef))
					break;
					
				bool endOfPath = (steerPosFlag & DT_STRAIGHTPATH_END) ? true : false;
				bool offMeshConnection = (steerPosFlag & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ? true : false;
					
				// Find movement delta.
				float delta[3], len;
				dtVsub(delta, steerPos, iterPos);
				len = dtMathSqrtf(dtVdot(delta, delta));
				// If the steer target is end of path or off-mesh link, do not move past the location.
				if ((endOfPath || offMeshConnection) && len < STEP_SIZE)
					len = 1;
				else
					len = STEP_SIZE / len;
				float moveTgt[3];
				dtVmad(moveTgt, iterPos, delta, len);
					
				// Move
				float result[3];
				dtPolyRef visited[16];
				int nvisited = 0;
				m_navQuery->moveAlongSurface(polys[0], iterPos, moveTgt, &m_filter,
												result, visited, &nvisited, 16);

				npolys = fixupCorridor(polys, npolys, MAX_POLYS, visited, nvisited);
				npolys = fixupShortcuts(polys, npolys, m_navQuery);

				float h = 0;
				dtStatus status = m_navQuery->getPolyHeight(polys[0], result, &h);
				if (dtStatusSucceed(status)) {
					result[1] = h;
				}
				dtVcopy(iterPos, result);

				// Handle end of path and off-mesh links when close enough.
				if (endOfPath && inRange(iterPos, steerPos, SLOP, 1.0f))
				{
					// Reached end of path.
					dtVcopy(iterPos, targetPos);
					if (m_nsmoothPath < MAX_SMOOTH)
					{
						dtVcopy(&path[m_nsmoothPath*3], iterPos);
						m_nsmoothPath++;
					}
					break;
				}
				else if (offMeshConnection && inRange(iterPos, steerPos, SLOP, 1.0f))
				{
					// Reached off-mesh connection.
					float startPos[3], endPos[3];
						
					// Advance the path up to and over the off-mesh connection.
					dtPolyRef prevRef = 0, polyRef = polys[0];
					int npos = 0;
					while (npos < npolys && polyRef != steerPosRef)
					{
						prevRef = polyRef;
						polyRef = polys[npos];
						npos++;
					}
					for (int i = npos; i < npolys; ++i)
						polys[i-npos] = polys[i];
					npolys -= npos;
						
					// Handle the connection.
					dtStatus status = m_navMesh->getOffMeshConnectionPolyEndPoints(prevRef, polyRef, startPos, endPos);
					if (dtStatusSucceed(status))
					{
						if (m_nsmoothPath < MAX_SMOOTH)
						{
							offmesh[*noffmesh] = m_nsmoothPath;
							(*noffmesh)++;
							dtVcopy(&path[m_nsmoothPath*3], startPos);
							m_nsmoothPath++;
							// Hack to make the dotted path not visible during off-mesh connection.
							/*if (m_nsmoothPath & 1)
							{
								dtVcopy(&path[m_nsmoothPath*3], startPos);
								m_nsmoothPath++;
							}*/
						}
						// Move position at the other side of the off-mesh link.
						dtVcopy(iterPos, endPos);
						float eh = 0.0f;
						m_navQuery->getPolyHeight(polys[0], iterPos, &eh);
						iterPos[1] = eh;
					}
				}
					
				// Store results.
				if (m_nsmoothPath < MAX_SMOOTH)
				{
					dtVcopy(&path[m_nsmoothPath*3], iterPos);
					m_nsmoothPath++;
				}
			}
		}
		*npath = m_nsmoothPath;
		return true;
	}
	else
	{
		m_npolys = 0;
		*npath = 0;
	}
	return false;
}

dtStatus Nav::findNearestPoly(const float* pos,dtPolyRef* ref,float* nearestPos){
	return m_navQuery->findNearestPoly(pos, m_polyPickExt, &m_filter, ref, nearestPos);
}

bool Nav::getNearestPos(const float* pos, float* nearestPos){
	bool bFound = false;
	dtPolyRef nearestRef = 0;
	dtStatus status = m_navQuery->findNearestPoly(pos, m_polyPickExt, &m_filter, &nearestRef, nearestPos);
	if(m_navQuery->isValidPolyRef(nearestRef,&m_filter))
	{
		bFound = true;
	}
	return bFound;
}

bool Nav::isWalkable(const float* pos,float maxDist, float* h,float* dist){
	float nearestPos[3];
	bool bFound = false;
	dtPolyRef nearestRef = 0;
	dtStatus status = m_navQuery->findNearestPoly(pos, m_polyPickExt, &m_filter, &nearestRef, nearestPos);
	if(dtStatusSucceed(status) && m_navQuery->isValidPolyRef(nearestRef,&m_filter))
	{
		float x3 = nearestPos[0] - pos[0];
		float y3 = 0;
		float z3 = nearestPos[2] - pos[2];
		*dist = sqrt(x3*x3+y3*y3+z3*z3);
		if(*dist < maxDist){
			bFound = true;
			status = m_navQuery->getPolyHeight(nearestRef,pos,h);
		}
	}
	return bFound;
}

Crowd::Crowd():
	m_nav(0),
	m_crowd(0),
	m_targetRef(0){
	m_crowd = dtAllocCrowd();
}

Crowd::~Crowd(){
	dtFreeCrowd(m_crowd);
}

void Crowd::init(Nav* nav){
	m_nav = nav;
	m_crowd->init(MAX_AGENTS,nav->getAgentRadius(),nav->getNavMesh());
	dtObstacleAvoidanceParams params;
	memcpy(&params, m_crowd->getObstacleAvoidanceParams(0), sizeof(dtObstacleAvoidanceParams));
	// Low (11)
	params.velBias = 0.5f;
	params.adaptiveDivs = 5;
	params.adaptiveRings = 2;
	params.adaptiveDepth = 1;
	m_crowd->setObstacleAvoidanceParams(0, &params);
		
	// Medium (22)
	params.velBias = 0.5f;
	params.adaptiveDivs = 5; 
	params.adaptiveRings = 2;
	params.adaptiveDepth = 2;
	m_crowd->setObstacleAvoidanceParams(1, &params);
		
	// Good (45)
	params.velBias = 0.5f;
	params.adaptiveDivs = 7;
	params.adaptiveRings = 2;
	params.adaptiveDepth = 3;
	m_crowd->setObstacleAvoidanceParams(2, &params);
		
	// High (66)
	params.velBias = 0.5f;
	params.adaptiveDivs = 7;
	params.adaptiveRings = 3;
	params.adaptiveDepth = 3;
	m_crowd->setObstacleAvoidanceParams(3, &params);
	printf("Crowd init DONE:%d\n",m_crowd->getAgentCount());
}

void Crowd::updateTick(const float dt){
	TimeVal startTime = getPerfTime();
	m_crowd->update(dt,NULL);
	TimeVal endTime = getPerfTime();

	for (int i = 0; i < m_crowd->getAgentCount(); ++i)
	{
		const dtCrowdAgent* ag = m_crowd->getAgent(i);
		if (!ag->active) continue;
		if(dtVdist(ag->npos,ag->targetPos) < 0.05f){
			m_crowd->resetMoveTarget(i);
		}
		if (ag->targetState == DT_CROWDAGENT_TARGET_VALID){
			//printf("updateTick:id=%d,pos=(%.2f,%.2f,%.2f),state=%d,targetState=%d\n",i,ag->npos[0],ag->npos[1],ag->npos[2],ag->state,ag->targetState);
		}
	}
}

int Crowd::addAgent(const float* pos){
	dtCrowdAgentParams ap;
	memset(&ap,0,sizeof(ap));
	ap.radius = m_nav->getAgentRadius();
	ap.height = m_nav->getAgentHeight();
	ap.maxAcceleration = 8.0f;
	ap.maxSpeed = 3.5f;
	ap.collisionQueryRange = ap.radius * 12.0f;
	ap.pathOptimizationRange = ap.radius * 30.0f;
	ap.updateFlags = DT_CROWD_OBSTACLE_AVOIDANCE;
	ap.obstacleAvoidanceType = 3;
	ap.separationWeight = 2.0f;

	int idx = m_crowd->addAgent(pos,&ap);
	if(idx != -1){
		if (m_targetRef)
			m_crowd->requestMoveTarget(idx,m_targetRef,m_targetPos);
	}
	const dtCrowdAgent* ag = m_crowd->getAgent(idx);
	return idx;
}

void Crowd::removeAgent(const int idx){
	m_crowd->removeAgent(idx);
}

static void calcVel(float* vel, const float* pos, const float* tgt, const float speed)
{
	dtVsub(vel, tgt, pos);
	vel[1] = 0.0;
	dtVnormalize(vel);
	dtVscale(vel, vel, speed);
}

bool Crowd::setMoveTarget(int idx,const float* pos,bool adjust,float* targetPos){
	printf("setMoveTarget:idx=%d,pos=(%.2f,%.2f,%.2f),adjust=%d\n",idx,pos[0],pos[1],pos[2],adjust);
	dtNavMeshQuery* navquery = m_nav->getNavMeshQuery();
	const dtQueryFilter* filter = m_crowd->getFilter(0);
	const float* halfExtents = m_crowd->getQueryExtents();
	if(adjust){
		float vel[3];
		const dtCrowdAgent* ag = m_crowd->getAgent(idx);
		if(ag->active){
			calcVel(vel,ag->npos,pos,ag->params.maxSpeed);
			m_crowd->requestMoveVelocity(idx,vel);
			return true;
		}
	}else{
		dtStatus status = navquery->findNearestPoly(pos,halfExtents,filter,&m_targetRef,targetPos);
		//dtStatus status = m_nav->findNearestPoly(pos,&m_targetRef,m_targetPos);
		if(dtStatusFailed(status)){
			printf("setMoveTarget FAILED:idx=%d,pos=(%.2f,%.2f,%.2f)\n",idx,pos[0],pos[1],pos[2]);
			return false;
		}
		else{
			const dtCrowdAgent* ag = m_crowd->getAgent(idx);
			if(ag->active){
				bool flag = m_crowd->requestMoveTarget(idx,m_targetRef,targetPos);
				printf("setMoveTarget:idx=%d,ref=%d,pos=(%.2f,%.2f,%.2f),flag=%d\n",idx,m_targetRef,targetPos[0],targetPos[1],targetPos[2],flag);
				return flag;
			}
		}
	}
	return false;
}

void Crowd::getAgentPos(const int idx,float * pos,int* state,int* targetState){
	const dtCrowdAgent* ag = m_crowd->getAgent(idx);
	if(ag){
		dtVcopy(pos,ag->npos);
		*state = (int)ag->state;
		*targetState = (int)ag->targetState;
	}
}
#pragma warning( pop )