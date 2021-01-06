#pragma warning( push )
#pragma warning( disable : 4244)
#pragma warning( disable : 4800)

#include "lnav.h"
#include <math.h>

template < typename T >
int gc_del(lua_State*L) {
	delete (*reinterpret_cast<T**>(lua_touserdata(L, 1)));
	return 0;
}

template <>
int gc_del<BuildContext>(lua_State*L) {
	delete (*reinterpret_cast<BuildContext**>(luaL_checkudata(L, 1, "BuildContext")));
	return 0;
}

template <>
int gc_del<Nav>(lua_State*L) {
	delete (*reinterpret_cast<Nav**>(luaL_checkudata(L, 1, "Nav")));
	return 0;
}

template <>
int gc_del<Crowd>(lua_State*L) {
	delete (*reinterpret_cast<Crowd**>(luaL_checkudata(L, 1, "Crowd")));
	return 0;
}

template < typename T >
T* atnew(lua_State*L, const char* ct) {
	*reinterpret_cast<T**>(lua_newuserdata(L, sizeof(T*))) = new T;

	if (luaL_newmetatable(L, ct)) {
		static const luaL_Reg functions[] =
		{
			{ "__gc", gc_del<T> },
			{ NULL, NULL }
		};
		luaL_setfuncs(L, functions, 0);
		lua_pushvalue(L, -1);
		lua_setfield(L, -2, "__index");
	}
	lua_setmetatable(L, -2);

	return (*reinterpret_cast<T**>(luaL_checkudata(L, -1, ct)));
}
#define anew( L, C ) atnew<C>( L, #C );

template < typename T >
T* atget(lua_State*L, const char* ct) {
	return (*reinterpret_cast<T**>(luaL_checkudata(L, 1, ct)));
}
#define aget( L, C ) atget<C>( L, #C );

template < typename T >
void atdel(lua_State*L, const char* ct) {
	delete (*reinterpret_cast<T**>(luaL_checkudata(L, 1, ct)));
}
#define adel(L, C) atdel<C>(L, #C);


static inline void _paramErr(lua_State* L) {
	const char* strParamErrNative = "native function param error";
	luaL_error( L, "%s\n", strParamErrNative );
}

int lass_navCreateContext(lua_State* L) {
	int flag = lua_toboolean(L, 1);
	BuildContext* ctx = anew(L, BuildContext);
	ctx->enableLog((bool)flag);
	return 1;
}

int lass_navCloseContext(lua_State* L) {
	adel(L, BuildContext);
	return 0;
}

int lass_navCreateNav(lua_State* L) {
	BuildContext* ctx = aget(L, BuildContext);
	if (ctx == NULL) {
		_paramErr(L);
		return 0;
	}
	Nav* nav = anew(L, Nav);
	nav->setContext(ctx);
	return 1;
}

int lass_navCreateCrowd(lua_State* L) {
	Nav* nav = aget(L, Nav);
	if (nav == NULL) {
		_paramErr(L);
		return 0;
	}
	Crowd* crowd = anew(L, Crowd);
	crowd->init(nav);
	return 1;
}

int lass_navLoad(lua_State* L) {
	bool flag;

	Nav* nav = aget(L, Nav);
	if (nav == NULL) {
		_paramErr(L);
		return 0;
	}
	const char* path = luaL_checkstring(L, 2);
	flag = nav->load(path);
	if (!flag) {
		luaL_error( L, "load nav file failed\n");
		return 0;
	}
	flag = nav->handleBuild();
	if (!flag) {
		luaL_error( L, "handle build failed\n");
		return 0;
	}
	lua_pushnumber(L, nav->getTotalBuildTimeMs());
	return 1;
}

int lass_navCloseNav(lua_State* L) {
	adel(L, Nav);
	return 0;
}

int lass_navCloseCrowd(lua_State* L) {
	adel(L, Crowd);
	return 0;
}

int lass_navRandomPos(lua_State* L) {
	Nav* nav = aget(L, Nav);
	if (nav == NULL) {
		_paramErr(L);
		return 0;
	}
	float pos[3];
	nav->findRandomPos(pos);
	lua_newtable(L);
	lua_pushnumber(L, -1);
	lua_rawseti(L, -2, 0);
	pos[0] = -pos[0];
	for (int i = 0; i<3; i++) {
		lua_pushnumber(L, pos[i]);
		lua_rawseti(L, -2, i + 1);
	}
	return 1;
}

int lass_navHeight(lua_State* L) {
	Nav* nav = aget(L, Nav);
	if (nav == NULL) {
		_paramErr(L);
		return 0;
	}
	float spos[3];
	spos[0] = (float)luaL_checknumber(L, 2);
	spos[1] = (float)luaL_checknumber(L, 3);
	spos[2] = (float)luaL_checknumber(L, 4);
	spos[0] = -spos[0];
	float h = 0.0f;
	bool flag = nav->getHeight(spos, &h);
	lua_pushboolean(L, flag);
	lua_pushnumber(L, h);
	return 2;
}

int lass_navHeightByHit(lua_State* L) {
	Nav* nav = aget(L, Nav);
	if (nav == NULL) {
		_paramErr(L);
		return 0;
	}
	float spos[3];
	spos[0] = (float)luaL_checknumber(L, 2);
	spos[1] = (float)luaL_checknumber(L, 3);
	spos[2] = (float)luaL_checknumber(L, 4);
	spos[0] = -spos[0];
	float diff = (float)luaL_checknumber(L, 5);
	float h = 0.0f;
	bool flag = nav->getHeightByHit(spos, diff, &h);
	lua_pushboolean(L, flag);
	lua_pushnumber(L, h);
	return 2;
}

int lass_navHitPos(lua_State* L) {
	Nav* nav = aget(L, Nav);
	if (nav == NULL) {
		_paramErr(L);
		return 0;
	}
	float spos[3];
	spos[0] = (float)luaL_checknumber(L, 2);
	spos[1] = (float)luaL_checknumber(L, 3);
	spos[2] = (float)luaL_checknumber(L, 4);
	spos[0] = -spos[0];
	float epos[3];
	epos[0] = (float)luaL_checknumber(L, 5);
	epos[1] = (float)luaL_checknumber(L, 6);
	epos[2] = (float)luaL_checknumber(L, 7);
	epos[0] = -epos[0];

	float pos[3];
	bool flag = nav->getHitPos(spos, epos, pos);
	lua_pushboolean(L, flag);
	lua_pushnumber(L, -pos[0]);
	lua_pushnumber(L, pos[1]);
	lua_pushnumber(L, pos[2]);
	return 4;
}

int lass_navHit(lua_State* L) {
	Nav* nav = aget(L, Nav);
	if (nav == NULL) {
		_paramErr(L);
		return 0;
	}
	float spos[3];
	spos[0] = (float)luaL_checknumber(L, 2);
	spos[1] = (float)luaL_checknumber(L, 3);
	spos[2] = (float)luaL_checknumber(L, 4);
	spos[0] = -spos[0];
	float epos[3];
	epos[0] = (float)luaL_checknumber(L, 5);
	epos[1] = (float)luaL_checknumber(L, 6);
	epos[2] = (float)luaL_checknumber(L, 7);
	epos[0] = -epos[0];

	float dist = (float)luaL_checknumber(L, 8);

	float pos[3];
	bool flag = nav->getHitPos(spos, epos, pos);
	if (flag) {
		float hitX = -pos[0];
		float hitY = epos[1];
		float hitZ = pos[2];
		float endX = -epos[0];
		float endY = epos[1];
		float endZ = epos[2];
		float dx = hitX - endX;
		float dy = hitY - endY;
		float dz = hitZ - endZ;
		flag = (dx*dx + dy*dy + dz*dz) > (dist*dist);
	}
	lua_pushboolean(L, flag);
	return 1;
}

int lass_setSetting(lua_State* L) {
	Nav* nav = aget(L, Nav);
	if (nav == NULL) {
		_paramErr(L);
		return 0;
	}
	float cellSize = (float)luaL_checknumber(L, 2);
	float cellHeight = (float)luaL_checknumber(L, 3);
	float agentHeight = (float)luaL_checknumber(L, 4);
	float agentRadius = (float)luaL_checknumber(L, 5);
	float agentMaxClimb = (float)luaL_checknumber(L, 6);
	float agentMaxSlope = (float)luaL_checknumber(L, 7);
	nav->setSetting(cellSize, cellHeight, agentHeight, agentRadius, agentMaxClimb, agentMaxSlope);
	return 0;
}

int lass_navPath(lua_State* L) {
	Nav* nav = aget(L, Nav);
	if (nav == NULL) {
		_paramErr(L);
		return 0;
	}
	unsigned int pathFindType = (unsigned int)luaL_checknumber(L, 2);
	float spos[3];
	spos[0] = (float)luaL_checknumber(L, 3);
	spos[1] = (float)luaL_checknumber(L, 4);
	spos[2] = (float)luaL_checknumber(L, 5);
	spos[0] = -spos[0];
	float epos[3];
	epos[0] = (float)luaL_checknumber(L, 6);
	epos[1] = (float)luaL_checknumber(L, 7);
	epos[2] = (float)luaL_checknumber(L, 8);
	epos[0] = -epos[0];

	static const int MAX_POLYS = 256;
	static const int MAX_SMOOTH = 2048;

	float path[MAX_SMOOTH * 3];
	int npath = 0;
	int offmesh[MAX_SMOOTH];
	int noffmesh = 0;
	bool flag = nav->findPath(pathFindType, spos, epos, path, &npath, offmesh, &noffmesh);
	lua_pushboolean(L, flag);
	lua_newtable(L);
	lua_pushnumber(L, -1);
	lua_rawseti(L, -2, 0);
	npath *= 3;
	for (int i = 0; i<npath; i++) {
		lua_pushnumber(L, i % 3 == 0 ? -path[i] : path[i]);
		lua_rawseti(L, -2, i + 1);
	}

	lua_newtable(L);
	for (int i = 0; i < noffmesh; i++) {
		lua_pushnumber(L, offmesh[i]);
		lua_rawseti(L, -2, i + 1);
	}
	return 3;
}

int lass_navGetOffmeshLink(lua_State* L) {
	Nav* nav = aget(L, Nav);
	if (nav == NULL) {
		_paramErr(L);
		return 0;
	}
	InputGeom* geom = nav->getGeom();
	int count = geom->getOffMeshConnectionCount();
	if (count <= 0) return 0;

	const float* verts = geom->getOffMeshConnectionVerts();
	count *= 6;
	lua_newtable(L);
	for (int i = 0; i < count; i++) {
		lua_pushnumber(L, i % 3 == 0 ? -verts[i] : verts[i]);
		lua_rawseti(L, -2, i+1);
	}
	return 1;
}


int lass_navPathDistance(lua_State* L) {
	Nav* nav = aget(L, Nav);
	if (nav == NULL) {
		_paramErr(L);
		return 0;
	}
	unsigned int pathFindType = (unsigned int)luaL_checknumber(L, 2);
	float spos[3];
	spos[0] = (float)luaL_checknumber(L, 3);
	spos[1] = (float)luaL_checknumber(L, 4);
	spos[2] = (float)luaL_checknumber(L, 5);
	spos[0] = -spos[0];
	float epos[3];
	epos[0] = (float)luaL_checknumber(L, 6);
	epos[1] = (float)luaL_checknumber(L, 7);
	epos[2] = (float)luaL_checknumber(L, 8);
	epos[0] = -epos[0];

	bool needSqrt = (float)luaL_checknumber(L, 9) > 0;

	static const int MAX_POLYS = 256;
	static const int MAX_SMOOTH = 2048;

	float path[MAX_SMOOTH * 3];
	int npath = 0;
	int offmesh[MAX_SMOOTH];
	int noffmesh = 0;
	bool flag = nav->findPath(pathFindType, spos, epos, path, &npath, offmesh, &noffmesh);
	lua_pushboolean(L, flag);
	float dist = 0;
	if (flag) {
		npath *= 3;
		float x1, y1, z1, x2, y2, z2, x3, y3, z3;
		x1 = path[0];
		y1 = path[1];
		z1 = path[2];
		for (int i = 3; i<npath; i += 3) {
			x2 = path[i];
			y2 = path[i + 1];
			z2 = path[i + 2];
			x3 = x2 - x1;
			y3 = y2 - y1;
			z3 = z2 - z1;
			if (needSqrt) {
				dist += sqrt(x3*x3 + y3*y3 + z3*z3);
			}
			else {
				dist += x3*x3 + y3*y3 + z3*z3;
			}
			x1 = x2;
			y1 = y2;
			z1 = z2;
		}
	}
	lua_pushnumber(L, dist);
	return 2;
}

int lass_navCrowdUpdate(lua_State* L) {
	Crowd* crowd = aget(L, Crowd);
	if (crowd == NULL) {
		_paramErr(L);
		return 0;
	}
	float dt = (float)luaL_checknumber(L, 2);
	crowd->updateTick(dt);
	return 0;
}

int lass_navCrowdAddAgent(lua_State* L) {
	Crowd* crowd = aget(L, Crowd);
	if (crowd == NULL) {
		_paramErr(L);
		return 0;
	}
	float spos[3];
	spos[0] = (float)luaL_checknumber(L, 2);
	spos[1] = (float)luaL_checknumber(L, 3);
	spos[2] = (float)luaL_checknumber(L, 4);
	spos[0] = -spos[0];
	int idx = crowd->addAgent(spos);
	lua_pushinteger(L, idx);
	return 1;
}

int lass_navCrowdDelAgent(lua_State* L) {
	Crowd* crowd = aget(L, Crowd);
	if (crowd == NULL) {
		_paramErr(L);
		return 0;
	}
	int idx = luaL_checkinteger(L, 2);
	crowd->removeAgent(idx);
	return 0;
}

int lass_navCrowdSetTarget(lua_State* L) {
	Crowd* crowd = aget(L, Crowd);
	if (crowd == NULL) {
		_paramErr(L);
		return 0;
	}
	int idx = luaL_checkinteger(L, 2);
	float spos[3];
	spos[0] = (float)luaL_checknumber(L, 3);
	spos[1] = (float)luaL_checknumber(L, 4);
	spos[2] = (float)luaL_checknumber(L, 5);
	spos[0] = -spos[0];
	bool adjust = luaL_checkinteger(L, 6);
	float targetPos[3] = { 0,0,0 };
	bool flag = crowd->setMoveTarget(idx, spos, adjust, targetPos);
	lua_pushboolean(L, flag);
	lua_pushnumber(L, -targetPos[0]);
	lua_pushnumber(L, targetPos[1]);
	lua_pushnumber(L, targetPos[2]);
	return 4;
}

int lass_navCrowdGetPos(lua_State* L) {
	Crowd* crowd = aget(L, Crowd);
	if (crowd == NULL) {
		_paramErr(L);
		return 0;
	}
	int idx = luaL_checkinteger(L, 2);
	float pos[3] = { 0,0,0 };
	int state = 0;
	int targetState = 0;
	crowd->getAgentPos(idx, pos, &state, &targetState);
	lua_newtable(L);
	lua_pushnumber(L, -1);
	lua_rawseti(L, -2, 0);
	pos[0] = -pos[0];
	for (int i = 0; i<3; i++) {
		lua_pushnumber(L, pos[i]);
		lua_rawseti(L, -2, i + 1);
	}
	lua_pushinteger(L, state);
	lua_rawseti(L, -2, 3 + 1);
	lua_pushinteger(L, targetState);
	lua_rawseti(L, -2, 4 + 1);
	return 1;
}

int lass_navGetNearestPos(lua_State* L) {
	Nav* nav = aget(L, Nav);
	if (nav == NULL) {
		_paramErr(L);
		return 0;
	}
	float pos[3];
	pos[0] = (float)luaL_checknumber(L, 2);
	pos[1] = (float)luaL_checknumber(L, 3);
	pos[2] = (float)luaL_checknumber(L, 4);
	pos[0] = -pos[0];
	float nearestPos[3];
	bool bFound = nav->getNearestPos(pos, nearestPos);
	lua_pushboolean(L, bFound);
	if (bFound) {
		lua_newtable(L);
		nearestPos[0] = -nearestPos[0];
		for (int i = 0; i < 3; i++) {
			lua_pushnumber(L, nearestPos[i]);
			lua_rawseti(L, -2, i + 1);
		}
	}
	else
	{
		lua_pushnil(L);
	}
	return 2;
}

int lass_navIsWalkable(lua_State* L) {
	Nav* nav = aget(L, Nav);
	if (nav == NULL) {
		_paramErr(L);
		return 0;
	}
	float maxDist = (float)luaL_checknumber(L, 2);
	float pos[3];
	pos[0] = (float)luaL_checknumber(L, 3);
	pos[1] = (float)luaL_checknumber(L, 4);
	pos[2] = (float)luaL_checknumber(L, 5);
	pos[0] = -pos[0];
	float h = 0;
	float dist = 0;
	bool bFound = nav->isWalkable(pos, maxDist, &h, &dist);
	lua_pushboolean(L, bFound);
	lua_pushnumber(L, h);
	lua_pushnumber(L, dist);
	return 3;
}


extern "C" int luaopen_detour(lua_State* L) {
	luaL_checkversion(L);

	luaL_Reg l[] = {
		{ "navCreateContext", lass_navCreateContext },
		{ "navCreateNav"	, lass_navCreateNav },
		{ "navCreateCrowd"	, lass_navCreateCrowd },
		{ "navCloseContext"	, lass_navCloseContext },
		{ "navCloseNav"		, lass_navCloseNav },
		{ "navCloseCrowd"	, lass_navCloseCrowd },
		{ "navLoad"			, lass_navLoad },
		{ "navRandomPos"	, lass_navRandomPos },
		{ "navHeight"		, lass_navHeight },
		{ "navHeightByHit"	, lass_navHeightByHit },
		{ "navHitPos"		, lass_navHitPos },
		{ "navHit"			, lass_navHit },
		{ "navSetSetting"	, lass_setSetting },
		{ "navPath"			, lass_navPath },
		{ "navPathDistance"	, lass_navPathDistance },
		{ "navCrowdUpdate"	, lass_navCrowdUpdate },
		{ "navCrowdAddAgent", lass_navCrowdAddAgent },
		{ "navCrowdDelAgent", lass_navCrowdDelAgent },
		{ "navCrowdSetTarget", lass_navCrowdSetTarget },
		{ "navCrowdGetPos"	, lass_navCrowdGetPos },
		{ "navGetNearestPos", lass_navGetNearestPos },
		{ "navIsWalkable"	, lass_navIsWalkable },
		{ "navGetOffmeshLink",lass_navGetOffmeshLink },

		{ NULL, NULL },
	};
	luaL_newlib(L, l);

	return 1;
}

#pragma warning( pop )
