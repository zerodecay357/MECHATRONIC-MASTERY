function sysCall_init()
    -- Get object handles
    dummyHandle = sim.getObject('../manipSphere')
    tipHandle = sim.getObject('../tip')

    -- Initialize positions
    startPos = sim.getObjectPosition(dummyHandle, -1)
    endPos = {1, 1, 0.5}

    -- Time parameters
    duration = 30
    startTime = sim.getSimulationTime()

    -- Motion parameters (defaults)
    r = 0.1
    amplitude = 0.1
    frequency = 2
    speed = 0.2

    -- Motion mode
    motionMode = 'circle' -- 'sine', 'square', 'circle', 'reachPoint'
    ratio = 0.0
end

function sysCall_actuation()
    -- Read variables from Python API
    local newR = sim.getFloatProperty(sim.handle_scene, "signal.r", {
        noError = true
    })
    local newAmplitude = sim.getFloatProperty(sim.handle_scene, "signal.amplitude", {
        noError = true
    })
    local newFrequency = sim.getFloatProperty(sim.handle_scene, "signal.frequency", {
        noError = true
    })
    local newSpeed = sim.getFloatProperty(sim.handle_scene, "signal.speed", {
        noError = true
    })
    local newMode = sim.getStringProperty(sim.handle_scene, "signal.motionMode", {
        noError = true
    })

    -- Update parameters if set from Python
    if newMode then
        motionMode = newMode
    end
    if newR then
        r = newR
    end
    if newAmplitude then
        amplitude = newAmplitude
    end
    if newSpeed then
        speed = newSpeed
    end
    if newFrequency then
        frequency = newFrequency
    end

    -- Calculate motion progress
    local t = sim.getSimulationTime() - startTime
    ratio = math.min(t / duration, 1.0)

    -- Execute motion based on mode
    if motionMode == 'sine' then
        sine()
    elseif motionMode == 'square' then
        square()
    elseif motionMode == 'circle' then
        circle()
    elseif motionMode == 'reachPoint' then
        reachPoint()
    end

    -- Check reachability
    local dummyPOS = sim.getObjectPosition(dummyHandle, -1)
    local tipPOS = sim.getObjectPosition(tipHandle, -1)
    for i = 1, 3, 1 do
        if math.abs(dummyPOS[i] - tipPOS[i]) > 0.05 then
            print("Can't be reached")
            sim.pauseSimulation()
        end
    end

    -- Reset cycle when complete
    if ratio >= 1.0 then
        startTime = sim.getSimulationTime()
    end
end

function reachPoint()
    local deltaX = endPos[1] - startPos[1]
    local deltaY = endPos[2] - startPos[2]
    local deltaZ = endPos[3] - startPos[3]

    local pos = {startPos[1] + ratio * deltaX, startPos[2] + ratio * deltaY, startPos[3] + ratio * deltaZ}
    sim.setObjectPosition(dummyHandle, -1, pos)
end

function square()
    local pos
    if ratio < 0.25 then
        local ratiom = ratio / 0.25
        pos = {startPos[1] + ratiom * r, startPos[2], startPos[3]}
    elseif ratio < 0.5 then
        local ratiom = (ratio - 0.25) / 0.25
        pos = {startPos[1] + r, startPos[2] + ratiom * r, startPos[3]}
    elseif ratio < 0.75 then
        local ratiom = (ratio - 0.5) / 0.25
        pos = {startPos[1] + (1 - ratiom) * r, startPos[2] + r, startPos[3]}
    else
        local ratiom = (ratio - 0.75) / 0.25
        pos = {startPos[1], startPos[2] + (1 - ratiom) * r, startPos[3]}
    end
    sim.setObjectPosition(dummyHandle, -1, pos)
end

function sine()
    if ratio < 0.5 then
        x = startPos[1] + r * ratio * 2
    else
        x = startPos[1] + r * (1 - ratio) * 2
    end
    local z = startPos[3]
    local y = startPos[2] + amplitude * math.sin(2 * math.pi * ratio * frequency)
    sim.setObjectPosition(dummyHandle, -1, {x, y, z})
end

function circle()
    local pos = {startPos[1] - r * (1 - math.cos(2 * math.pi * ratio)), startPos[2] + r * math.sin(2 * math.pi * ratio),
                 startPos[3]}
    sim.setObjectPosition(dummyHandle, -1, pos)
end
