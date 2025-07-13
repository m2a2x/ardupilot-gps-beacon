-- beacon_arm.lua
-- Arms drone when BEACON_ARM == 1 and mode is GUIDED
-- Improved version following AP_Scripting examples patterns

-- Constants
local GUIDED_MODE = 4       -- Copter GUIDED mode
local check_interval_ms = 1000

-- Parameter table configuration
-- The table key must be unique (0-200) and not used by other scripts
local PARAM_TABLE_KEY = 73  -- Using 73 to avoid conflict with example (72)

-- Create parameter table with 3 parameters
-- Prefix "BEACON_" ensures no conflicts with other scripts
assert(param:add_table(PARAM_TABLE_KEY, "BEACON_", 3), 'could not add param table')

-- Add individual parameters to the table
-- Parameters are floats with default values
assert(param:add_param(PARAM_TABLE_KEY, 1, 'ARM_ENABLE', 0), 'could not add ARM_ENABLE param')
assert(param:add_param(PARAM_TABLE_KEY, 2, 'COOLDOWN_MS', 5000), 'could not add COOLDOWN_MS param')
assert(param:add_param(PARAM_TABLE_KEY, 3, 'DEBUG_LEVEL', 0), 'could not add DEBUG_LEVEL param')

-- Parameter objects for fast access (following param_get_set_test.lua pattern)
local beacon_arm_enable = Parameter()
if not beacon_arm_enable:init('BEACON_ARM_ENABLE') then
    gcs:send_text(6, 'Failed to initialize BEACON_ARM_ENABLE parameter')
    return
end

local beacon_cooldown = Parameter()
if not beacon_cooldown:init('BEACON_COOLDOWN_MS') then
    gcs:send_text(6, 'Failed to initialize BEACON_COOLDOWN_MS parameter')
    return
end

local beacon_debug = Parameter()
if not beacon_debug:init('BEACON_DEBUG_LEVEL') then
    gcs:send_text(6, 'Failed to initialize BEACON_DEBUG_LEVEL parameter')
    return
end

-- State tracking
local last_arming_attempt = 0
local script_initialized = false

function update()
    local current_time = millis()
    local mode = vehicle:get_mode()
    
    -- Initialize script on first run
    if not script_initialized then
        gcs:send_text(6, "Beacon arming script initialized")
        script_initialized = true
    end
    
    local arm_enable = beacon_arm_enable:get()
    local cooldown_ms = beacon_cooldown:get()
    local debug_level = beacon_debug:get()
    
    -- Check if parameters exist and have valid values
    if not arm_enable or not cooldown_ms or not debug_level then
        gcs:send_text(6, 'Failed to read beacon parameters')
        return update, check_interval_ms
    end

    -- Debug output if enabled
    if debug_level > 0 then
        gcs:send_text(6, string.format("Beacon: Enable=%d, Cooldown=%dms, Mode=%d", 
                                      arm_enable, cooldown_ms, mode))
    end

    -- Only run in GUIDED mode when beacon arming is enabled
    if mode == GUIDED_MODE and arm_enable == 1 then
        
        -- Check if already armed
        if not arming:is_armed() then
            
            -- Check cooldown to prevent rapid arming attempts
            if current_time - last_arming_attempt > cooldown_ms then
                gcs:send_text(6, "Beacon arming requested")
                last_arming_attempt = current_time

                -- Attempt to arm
                local result = arming:arm()

                if result then
                    gcs:send_text(6, "Drone armed via beacon")
                else
                    gcs:send_text(3, "Beacon arm failed - arming checks not passed")
                end

                -- Reset parameter to prevent repeat arming
                if not beacon_arm_enable:set(0) then
                    gcs:send_text(6, "Failed to reset BEACON_ARM_ENABLE parameter")
                end
            end
        else
            -- Already armed, reset the parameter
            if beacon_arm_enable:get() == 1 then
                if not beacon_arm_enable:set(0) then
                    gcs:send_text(6, "Failed to reset BEACON_ARM_ENABLE parameter")
                end
            end
        end
    end

    return update, check_interval_ms
end

return update()
