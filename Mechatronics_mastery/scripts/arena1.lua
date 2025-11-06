sim=require'sim'

-- ========================================
-- GLOBAL VARIABLES
-- ========================================
platform = nil
omniPads = {}
waypoints = {}
obstacles = {}
currentWaypointIndex = 1
proximitySensors = {}
startTime = 0
stuckTimer = 0
lastPosition = {x = 0, y = 0}

-- SMOOTH MOTION: Current velocity tracking
currentVelocity = {x = 0, y = 0}
targetVelocity = {x = 0, y = 0}

-- Configuration parameters
v_max = 2.0  -- Reduced max velocity for smoother motion
waypointThreshold = 0.25
obstacleAvoidanceRange = 0.6
minObstacleDistance = 0.30
wheelRadius = 0.04
robotRadius = 0.175

-- SMOOTH MOTION PARAMETERS (Enhanced for less jitter)
maxAcceleration = 2.5  -- Reduced for smoother acceleration
smoothingFactor = 0.08  -- Lower value = smoother motion (reduced from 0.15)

-- ARENA BOUNDARIES
arenaBounds = {
    minX = -2.2,
    maxX = 2.2,
    minY = -2.2,
    maxY = 2.2
}
boundaryMargin = 0.3

-- A* Path Planning Variables
gridResolution = 0.15  -- Grid cell size for A* (meters)
inflationRadius = 0.25  -- Obstacle inflation for safety margin
pathSmoothing = true
currentPathIndex = 1
globalPath = {}

-- Performance tracking
waypointsReached = 0
collisionCount = 0
boundaryWarnings = 0
stuckCount = 0

-- ========================================
-- A* ALGORITHM IMPLEMENTATION
-- ========================================

-- Node structure for A* algorithm
function createNode(x, y, parent, g, h)
    return {
        x = x,
        y = y,
        parent = parent,
        g = g,  -- Cost from start
        h = h,  -- Heuristic to goal
        f = g + h  -- Total cost
    }
end

-- Heuristic function (Euclidean distance)
function heuristic(x1, y1, x2, y2)
    return math.sqrt((x2 - x1)^2 + (y2 - y1)^2)
end

-- Convert world coordinates to grid coordinates
function worldToGrid(x, y)
    local gx = math.floor((x - arenaBounds.minX) / gridResolution)
    local gy = math.floor((y - arenaBounds.minY) / gridResolution)
    return gx, gy
end

-- Convert grid coordinates to world coordinates (center of cell)
function gridToWorld(gx, gy)
    local x = arenaBounds.minX + (gx + 0.5) * gridResolution
    local y = arenaBounds.minY + (gy + 0.5) * gridResolution
    return x, y
end

-- Check if a grid cell is valid (not obstacle, within bounds)
function isValidCell(gx, gy, obstacleGrid)
    local gridWidth = math.ceil((arenaBounds.maxX - arenaBounds.minX) / gridResolution)
    local gridHeight = math.ceil((arenaBounds.maxY - arenaBounds.minY) / gridResolution)
    
    if gx < 0 or gx >= gridWidth or gy < 0 or gy >= gridHeight then
        return false
    end
    
    return not obstacleGrid[gx .. "," .. gy]
end

-- Create obstacle grid with inflation
function createObstacleGrid()
    local obstacleGrid = {}
    local gridWidth = math.ceil((arenaBounds.maxX - arenaBounds.minX) / gridResolution)
    local gridHeight = math.ceil((arenaBounds.maxY - arenaBounds.minY) / gridResolution)
    
    -- Mark obstacle cells with inflation
    for _, obs in ipairs(obstacles) do
        local ogx, ogy = worldToGrid(obs.x, obs.y)
        local inflationCells = math.ceil((obs.radius + inflationRadius) / gridResolution)
        
        for dx = -inflationCells, inflationCells do
            for dy = -inflationCells, inflationCells do
                local gx = ogx + dx
                local gy = ogy + dy
                local wx, wy = gridToWorld(gx, gy)
                local dist = math.sqrt((wx - obs.x)^2 + (wy - obs.y)^2)
                
                if dist <= (obs.radius + inflationRadius) then
                    obstacleGrid[gx .. "," .. gy] = true
                end
            end
        end
    end
    
    return obstacleGrid
end

-- A* pathfinding algorithm
function aStarPathfinding(startX, startY, goalX, goalY)
    local obstacleGrid = createObstacleGrid()
    
    local startGx, startGy = worldToGrid(startX, startY)
    local goalGx, goalGy = worldToGrid(goalX, goalY)
    
    -- Check if start or goal is in obstacle
    if not isValidCell(startGx, startGy, obstacleGrid) then
        print("? Start position is in obstacle!")
        return nil
    end
    if not isValidCell(goalGx, goalGy, obstacleGrid) then
        print("? Goal position is in obstacle!")
        return nil
    end
    
    local openSet = {}
    local closedSet = {}
    local openSetHash = {}
    
    local startNode = createNode(startGx, startGy, nil, 0, heuristic(startGx, startGy, goalGx, goalGy))
    table.insert(openSet, startNode)
    openSetHash[startGx .. "," .. startGy] = true
    
    local directions = {
        {0, 1}, {1, 0}, {0, -1}, {-1, 0},  -- 4-connected
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1}  -- Diagonals
    }
    
    local iterations = 0
    local maxIterations = 5000
    
    while #openSet > 0 and iterations < maxIterations do
        iterations = iterations + 1
        
        -- Find node with lowest f cost
        local currentIndex = 1
        for i = 2, #openSet do
            if openSet[i].f < openSet[currentIndex].f then
                currentIndex = i
            end
        end
        
        local current = table.remove(openSet, currentIndex)
        openSetHash[current.x .. "," .. current.y] = nil
        closedSet[current.x .. "," .. current.y] = true
        
        -- Check if goal reached
        if current.x == goalGx and current.y == goalGy then
            local path = {}
            local node = current
            while node do
                local wx, wy = gridToWorld(node.x, node.y)
                table.insert(path, 1, {x = wx, y = wy})
                node = node.parent
            end
            print(string.format("? A* path found with %d waypoints (%d iterations)", #path, iterations))
            return path
        end
        
        -- Explore neighbors
        for _, dir in ipairs(directions) do
            local nx = current.x + dir[1]
            local ny = current.y + dir[2]
            
            if isValidCell(nx, ny, obstacleGrid) and not closedSet[nx .. "," .. ny] then
                local isDiagonal = dir[1] ~= 0 and dir[2] ~= 0
                local moveCost = isDiagonal and 1.414 or 1.0
                local newG = current.g + moveCost
                
                local neighborKey = nx .. "," .. ny
                if not openSetHash[neighborKey] then
                    local h = heuristic(nx, ny, goalGx, goalGy)
                    local neighbor = createNode(nx, ny, current, newG, h)
                    table.insert(openSet, neighbor)
                    openSetHash[neighborKey] = true
                else
                    -- Update if better path found
                    for i, node in ipairs(openSet) do
                        if node.x == nx and node.y == ny and newG < node.g then
                            node.g = newG
                            node.f = newG + node.h
                            node.parent = current
                            break
                        end
                    end
                end
            end
        end
    end
    
    print("? A* failed to find path!")
    return nil
end

-- Smooth path using simple averaging
function smoothPath(path)
    if not path or #path <= 2 then return path end
    
    local smoothed = {path[1]}
    
    for i = 2, #path - 1 do
        local prev = path[i - 1]
        local curr = path[i]
        local next = path[i + 1]
        
        -- Average with neighbors
        local smoothX = (prev.x + curr.x * 2 + next.x) / 4
        local smoothY = (prev.y + curr.y * 2 + next.y) / 4
        
        table.insert(smoothed, {x = smoothX, y = smoothY})
    end
    
    table.insert(smoothed, path[#path])
    return smoothed
end

-- ========================================
-- AUTOMATIC ARENA BOUNDARY DETECTION
-- ========================================
function measureArenaBoundaries()
    print("\n=== MEASURING ARENA BOUNDARIES ===")
    
    local arenaNames = {'/Floor', '/Arena', '/Platform', '/floor', '/arena', 
                       '/platform', '/CustomizableArena_5_25', '/resizableFloor_5_25',
                       '/Floor_5_25', 'Floor', 'Arena', 'Platform'}
    local arenaHandle = nil
    local foundName = ""
    
    for _, name in ipairs(arenaNames) do
        local result, handle = pcall(sim.getObject, name)
        if result then
            arenaHandle = handle
            foundName = name
            print(string.format("? Found arena: %s", name))
            break
        end
    end
    
    if arenaHandle then
        local result, xMin = sim.getObjectFloatParameter(arenaHandle, sim.objfloatparam_objbbox_min_x)
        local result, xMax = sim.getObjectFloatParameter(arenaHandle, sim.objfloatparam_objbbox_max_x)
        local result, yMin = sim.getObjectFloatParameter(arenaHandle, sim.objfloatparam_objbbox_min_y)
        local result, yMax = sim.getObjectFloatParameter(arenaHandle, sim.objfloatparam_objbbox_max_y)
        
        local arenaPos = sim.getObjectPosition(arenaHandle, -1)
        
        local globalXMin = arenaPos[1] + xMin
        local globalXMax = arenaPos[1] + xMax
        local globalYMin = arenaPos[2] + yMin
        local globalYMax = arenaPos[2] + yMax
        
        print("\n?? ARENA MEASUREMENTS:")
        print(string.format("  X Range: %.3f to %.3f (width: %.3f m)", globalXMin, globalXMax, globalXMax - globalXMin))
        print(string.format("  Y Range: %.3f to %.3f (length: %.3f m)", globalYMin, globalYMax, globalYMax - globalYMin))
        
        return {
            minX = globalXMin,
            maxX = globalXMax,
            minY = globalYMin,
            maxY = globalYMax
        }
    else
        print("? WARNING: Could not auto-detect arena!")
        return nil
    end
end

-- ========================================
-- INITIALIZATION
-- ========================================
function sysCall_init()
    print("=== Holonomic Robot with A* Path Planning ===")
    print("Initializing robot...")
    
    platform = sim.getObject('..')
    
    for i = 1, 4, 1 do
        omniPads[i] = sim.getObject('../link['..(i-1)..']/regularRotation')
    end
    
    local startPosition = sim.getObjectPosition(platform, -1)
    print(string.format("Start Position: (%.2f, %.2f, %.2f)", 
          startPosition[1], startPosition[2], startPosition[3]))
    
    lastPosition.x = startPosition[1]
    lastPosition.y = startPosition[2]
    
    -- Initialize velocity to zero
    currentVelocity = {x = 0, y = 0}
    targetVelocity = {x = 0, y = 0}
    
    -- Auto-detect arena boundaries
    local measuredBounds = measureArenaBoundaries()
    if measuredBounds then
        arenaBounds = measuredBounds
        print("\n? Arena boundaries detected!")
    end
    
    print(string.format("\n?? Arena Bounds: X[%.2f to %.2f], Y[%.2f to %.2f]\n", 
          arenaBounds.minX, arenaBounds.maxX, arenaBounds.minY, arenaBounds.maxY))
    
    -- Original waypoints
    waypoints = {
        {x = 1.575, y = 0.9, z = 0},
        {x = -1.425, y = 1.525, z = 0},
        {x = -0.125, y = -0.075, z = 0},
        {x = 0.225, y = -1.65, z = 0},
        {x = -1.85, y = -0.7, z = 0}
    }
    
    print(string.format("Total Target Waypoints: %d", #waypoints))
    
    setupProximitySensors()
    detectStaticObstacles()
    
    -- Plan path using A* from start to first waypoint
    print("\n=== PLANNING PATH WITH A* ===")
    planPathToNextWaypoint(startPosition[1], startPosition[2])
    
    startTime = sim.getSimulationTime()
    stuckTimer = startTime
    
    print("\n? Initialization complete!")
    print("?? A* path planning enabled for obstacle avoidance\n")
end

-- ========================================
-- PATH PLANNING HELPER
-- ========================================
function planPathToNextWaypoint(fromX, fromY)
    if currentWaypointIndex > #waypoints then
        return
    end
    
    local target = waypoints[currentWaypointIndex]
    print(string.format("Planning path to Waypoint %d: (%.2f, %.2f)", 
          currentWaypointIndex, target.x, target.y))
    
    local path = aStarPathfinding(fromX, fromY, target.x, target.y)
    
    if path then
        if pathSmoothing then
            path = smoothPath(path)
            print(string.format("  Path smoothed to %d points", #path))
        end
        globalPath = path
        currentPathIndex = 1
    else
        print("? A* failed! Using direct path")
        globalPath = {{x = target.x, y = target.y}}
        currentPathIndex = 1
    end
end

-- ========================================
-- SMOOTH VELOCITY INTERPOLATION
-- ========================================
function smoothVelocity(current, target, smoothing)
    return {
        x = current.x + (target.x - current.x) * smoothing,
        y = current.y + (target.y - current.y) * smoothing
    }
end

-- ========================================
-- SENSOR SETUP
-- ========================================
function setupProximitySensors()
    proximitySensors = {}
    
    for i = 0, 15 do
        local sensorName = '../proxSensor'
        if i > 0 then
            sensorName = '../proxSensor[' .. (i-1) .. ']'
        end
        
        local result, handle = pcall(sim.getObject, sensorName)
        if result then
            table.insert(proximitySensors, {
                handle = handle, 
                angle = (i * 2 * math.pi) / 16
            })
        end
    end
    
    print(string.format("? Proximity sensors: %d", #proximitySensors))
end

-- ========================================
-- OBSTACLE DETECTION
-- ========================================
function detectStaticObstacles()
    obstacles = {}
    
    local allObjects = sim.getObjectsInTree(sim.handle_scene, sim.object_shape_type, 0)
    
    for i = 1, #allObjects do
        local objHandle = allObjects[i]
        local objName = sim.getObjectAlias(objHandle, 2)
        
        -- Detect obstacles (including Waypoint objects that might be obstacles)
        if objName and (string.find(string.lower(objName), "obstacle") or 
                        string.find(string.lower(objName), "obst")) then
            local pos = sim.getObjectPosition(objHandle, -1)
            
            table.insert(obstacles, {
                handle = objHandle,
                x = pos[1], 
                y = pos[2], 
                z = pos[3],
                radius = 0.32
            })
            
            print(string.format("  Obstacle: %s at (%.2f, %.2f)", objName, pos[1], pos[2]))
        end
    end
    
    print(string.format("? Obstacles detected: %d", #obstacles))
end

-- ========================================
-- STUCK DETECTION AND RECOVERY
-- ========================================
function checkIfStuck(robotPos)
    local currentTime = sim.getSimulationTime()
    
    local dx = robotPos[1] - lastPosition.x
    local dy = robotPos[2] - lastPosition.y
    local distanceMoved = math.sqrt(dx * dx + dy * dy)
    
    if (currentTime - stuckTimer) > 3.0 then
        if distanceMoved < 0.15 then
            stuckCount = stuckCount + 1
            print(string.format("? STUCK DETECTED! (#%d) - Replanning path...", stuckCount))
            -- Replan path when stuck
            planPathToNextWaypoint(robotPos[1], robotPos[2])
            return true
        end
        
        stuckTimer = currentTime
        lastPosition.x = robotPos[1]
        lastPosition.y = robotPos[2]
    end
    
    return false
end

-- ========================================
-- BOUNDARY CHECK
-- ========================================
function checkBoundaries(robotPos)
    local boundaryAvoidX = 0
    local boundaryAvoidY = 0
    local nearBoundary = false
    
    local distToLeftEdge = robotPos[1] - arenaBounds.minX
    local distToRightEdge = arenaBounds.maxX - robotPos[1]
    
    if distToLeftEdge < boundaryMargin then
        local force = (boundaryMargin - distToLeftEdge) / boundaryMargin
        boundaryAvoidX = force * 2.0
        nearBoundary = true
    elseif distToRightEdge < boundaryMargin then
        local force = (boundaryMargin - distToRightEdge) / boundaryMargin
        boundaryAvoidX = -force * 2.0
        nearBoundary = true
    end
    
    local distToBottomEdge = robotPos[2] - arenaBounds.minY
    local distToTopEdge = arenaBounds.maxY - robotPos[2]
    
    if distToBottomEdge < boundaryMargin then
        local force = (boundaryMargin - distToBottomEdge) / boundaryMargin
        boundaryAvoidY = force * 2.0
        nearBoundary = true
    elseif distToTopEdge < boundaryMargin then
        local force = (boundaryMargin - distToTopEdge) / boundaryMargin
        boundaryAvoidY = -force * 2.0
        nearBoundary = true
    end
    
    if nearBoundary then
        boundaryWarnings = boundaryWarnings + 1
    end
    
    return boundaryAvoidX, boundaryAvoidY, nearBoundary
end

-- ========================================
-- MAIN ACTUATION LOOP (WITH A* AND SMOOTH MOTION)
-- ========================================
function sysCall_actuation()
    if currentWaypointIndex > #waypoints then
        -- Smooth deceleration to stop
        targetVelocity.x = 0
        targetVelocity.y = 0
        currentVelocity = smoothVelocity(currentVelocity, targetVelocity, smoothingFactor)
        
        -- Apply smoothed velocity
        setOmniWheelVelocities(currentVelocity.x, currentVelocity.y, 0)
        
        -- Check if stopped
        if math.abs(currentVelocity.x) < 0.01 and math.abs(currentVelocity.y) < 0.01 then
            stopRobot()
            
            if waypointsReached == #waypoints then
                local totalTime = sim.getSimulationTime() - startTime
                print("\n" .. string.rep("=", 50))
                print("?? NAVIGATION COMPLETE!")
                print(string.rep("=", 50))
                print(string.format("? Waypoints: %d/%d", waypointsReached, #waypoints))
                print(string.format("? Time: %.2fs", totalTime))
                print(string.format("? Near-collisions: %d", collisionCount))
                print(string.format("?? Stuck recoveries: %d", stuckCount))
                print(string.rep("=", 50) .. "\n")
            end
        end
        
        return
    end
    
    local robotPos = sim.getObjectPosition(platform, -1)
    local robotOri = sim.getObjectOrientation(platform, -1)
    
    local isStuck = checkIfStuck(robotPos)
    
    -- Follow A* path
    if currentPathIndex <= #globalPath then
        local pathPoint = globalPath[currentPathIndex]
        
        local dx = pathPoint.x - robotPos[1]
        local dy = pathPoint.y - robotPos[2]
        local distToPathPoint = math.sqrt(dx * dx + dy * dy)
        
        -- Move to next path point if close enough
        if distToPathPoint < 0.2 then
            currentPathIndex = currentPathIndex + 1
            if currentPathIndex > #globalPath then
                -- Reached final waypoint
                waypointsReached = waypointsReached + 1
                print(string.format("\n? Waypoint %d REACHED! (%.2fs)", 
                      currentWaypointIndex, sim.getSimulationTime() - startTime))
                currentWaypointIndex = currentWaypointIndex + 1
                stuckTimer = sim.getSimulationTime()
                
                -- Plan path to next waypoint
                if currentWaypointIndex <= #waypoints then
                    planPathToNextWaypoint(robotPos[1], robotPos[2])
                end
                return
            end
        end
        
        -- Calculate goal-seeking direction
        local desiredVx = dx / distToPathPoint
        local desiredVy = dy / distToPathPoint
        
        local boundaryX, boundaryY, nearBoundary = checkBoundaries(robotPos)
        local avoidX, avoidY = calculateObstacleAvoidance(robotPos, robotOri)
        
        -- Combine behaviors (reduced obstacle avoidance since A* handles it)
        local finalVx, finalVy
        
        if isStuck then
            finalVx = -desiredVy + math.random(-30, 30) / 100
            finalVy = desiredVx + math.random(-30, 30) / 100
            stuckTimer = sim.getSimulationTime()
        elseif nearBoundary then
            finalVx = desiredVx * 0.3 + boundaryX * 0.5 + avoidX * 0.2
            finalVy = desiredVy * 0.3 + boundaryY * 0.5 + avoidY * 0.2
        elseif avoidX ~= 0 or avoidY ~= 0 then
            -- Less aggressive avoidance since path is already planned
            finalVx = desiredVx * 0.7 + avoidX * 0.3
            finalVy = desiredVy * 0.7 + avoidY * 0.3
        else
            finalVx = desiredVx
            finalVy = desiredVy
        end
        
        -- Normalize
        local velMag = math.sqrt(finalVx * finalVx + finalVy * finalVy)
        if velMag > 0.001 then
            finalVx = finalVx / velMag
            finalVy = finalVy / velMag
        else
            finalVx = 0
            finalVy = 0
        end
        
        -- Speed control with smooth deceleration
        local speed = v_max
        
        if isStuck then
            speed = v_max * 0.6
        elseif distToPathPoint < 0.8 then
            -- Smoother deceleration curve
            local slowdownFactor = math.pow(distToPathPoint / 0.8, 0.7)
            speed = v_max * math.max(0.3, slowdownFactor)
        end
        
        if nearBoundary then
            speed = speed * 0.5
        elseif avoidX ~= 0 or avoidY ~= 0 then
            speed = speed * 0.7
        end
        
        -- Set TARGET velocity
        targetVelocity.x = finalVx * speed
        targetVelocity.y = finalVy * speed
        
        -- SMOOTH TRANSITION with lower smoothing factor
        currentVelocity = smoothVelocity(currentVelocity, targetVelocity, smoothingFactor)
        
        -- Apply the SMOOTHED velocity to wheels
        setOmniWheelVelocities(currentVelocity.x, currentVelocity.y, 0)
    end
end

-- ========================================
-- OBSTACLE AVOIDANCE (Reduced influence with A*)
-- ========================================
function calculateObstacleAvoidance(robotPos, robotOri)
    local avoidX = 0
    local avoidY = 0
    
    -- Proximity sensor based avoidance (for dynamic obstacles)
    if #proximitySensors > 0 then
        for i, sensor in ipairs(proximitySensors) do
            local result, distance, detectedPoint = sim.readProximitySensor(sensor.handle)
            
            if result > 0 and distance < obstacleAvoidanceRange then
                local repulsion = math.pow((obstacleAvoidanceRange - distance) / obstacleAvoidanceRange, 2) * 1.2
                local sensorGlobalAngle = robotOri[3] + sensor.angle
                
                avoidX = avoidX - math.cos(sensorGlobalAngle) * repulsion
                avoidY = avoidY - math.sin(sensorGlobalAngle) * repulsion
                
                if distance < minObstacleDistance then
                    collisionCount = collisionCount + 1
                end
            end
        end
    end
    
    -- Normalize avoidance vector
    if avoidX ~= 0 or avoidY ~= 0 then
        local avoidMag = math.sqrt(avoidX * avoidX + avoidY * avoidY)
        if avoidMag > 0 then
            avoidX = avoidX / avoidMag
            avoidY = avoidY / avoidMag
        end
    end
    
    return avoidX, avoidY
end

-- ========================================
-- OMNI-WHEEL KINEMATICS
-- ========================================
function setOmniWheelVelocities(vx, vy, omega)
    local v1 = (vx - vy - robotRadius * omega) / wheelRadius
    local v2 = (vx + vy + robotRadius * omega) / wheelRadius
    local v3 = (vx - vy + robotRadius * omega) / wheelRadius
    local v4 = (vx + vy - robotRadius * omega) / wheelRadius
    
    sim.setJointTargetVelocity(omniPads[1], -v1)
    sim.setJointTargetVelocity(omniPads[2], -v2)
    sim.setJointTargetVelocity(omniPads[3], v3)
    sim.setJointTargetVelocity(omniPads[4], v4)
end

-- ========================================
-- UTILITY FUNCTIONS
-- ========================================
function stopRobot()
    for i = 1, 4 do
        sim.setJointTargetVelocity(omniPads[i], 0)
    end
end

function sysCall_sensing()
end

function sysCall_cleanup()
    print("\n=== Cleanup ===")
    stopRobot()
    print(string.format("Waypoints: %d/%d", waypointsReached, #waypoints))
    print(string.format("Stuck recoveries: %d", stuckCount))
end
