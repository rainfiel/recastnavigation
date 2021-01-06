
local detour = require "detour"

local c = detour.navCreateContext(true)
local n = detour.navCreateNav(c)

local fullPath = "Meshes/objlv0.gset"
detour.navLoad(n,fullPath)


local registry = debug.getregistry()
registry.ud_nav = n

local links = detour.navGetOffmeshLink(n)
print(links)
if links then
    print(table.concat(links, ","))
end

local flag, poses, offmesh = detour.navPath(n, 1, 38.71, 5.79, 33, 29, 2, 30)
print(flag, poses, offmesh)
print(table.concat(offmesh, ";"))
print(table.concat(poses, ";"))

local idx = offmesh[1]
print(poses[idx * 3 + 1], poses[idx * 3 + 2], poses[idx * 3 + 3])

print("main loaded")