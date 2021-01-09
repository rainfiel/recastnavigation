
local detour = require "detour"

local c = detour.navCreateContext(true)
local n = detour.navCreateNav(c)

local fullPath = "Meshes/objlv3.gset"
detour.navLoad(n,fullPath)


local registry = debug.getregistry()
registry.ud_nav = n

local links = detour.navGetOffmeshLink(n)
print(links)
if links then
    print(table.concat(links, ","))
end

local flag, poses, offmesh = detour.navPath(n, 1, 45,2,79, 47,0,61)
print(table.concat(offmesh, ";"))
print(table.concat(poses, ";"))

detour.navSetExcludeFilter(n, 32)
local flag, poses, offmesh = detour.navPath(n, 1, 45,2,79, 47,0,61)
print(table.concat(offmesh, ";"))
print(table.concat(poses, ";"))

print(".........:", detour.navGetIncludeFilter(n), detour.navGetNavCost(n, 5))

-- detour.navSetExcludeFilter(n, 0)
-- local flag, poses, offmesh = detour.navPath(n, 1, 36,0.5,65, -10,2,72)
-- print(table.concat(offmesh, ";"))
-- print(table.concat(poses, ";"))

-- local idx = offmesh[1]
-- print(poses[idx * 3 + 1], poses[idx * 3 + 2], poses[idx * 3 + 3])

print("main loaded")