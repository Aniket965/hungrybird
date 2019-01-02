-- This script is used for realtime emulation of the environment in V-REP

function sysCall_init()

    -- Add required handles here


    
     drone = sim.getObjectHandle('Drone_Pos_Emulation')

     -- Subscribing to the required topic
     whycon_sub = simROS.subscribe('/whycon/poses', 'geometry_msgs/PoseArray', 'whycon_callback')
end


function sysCall_actuation()

end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end


function whycon_callback(msg)
    -- Get the position of the real-world whycon marker and set the position of the drone in the simulator.
     pos = msg.poses[1].position
     sim.setObjectPosition(drone,-1,{-pos.x/4,pos.y/4,(pos.z - 35)/-17.32}) 
end
