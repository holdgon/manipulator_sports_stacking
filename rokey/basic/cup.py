import rclpy
import DR_init

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_test", namespace=ROBOT_ID)

    DR_init.__dsr__node = node
    
    ON, OFF = 1, 0
    
    from DSR_ROBOT2 import (
        DR_BASE,
        DR_TOOL,
        DR_MV_MOD_REL,
        DR_MV_MOD_ABS,
        set_desired_force,
        DR_FC_MOD_REL,
        task_compliance_ctrl,
        release_compliance_ctrl,
        check_force_condition,
        DR_AXIS_Z,
        set_digital_output,
        posx,
        posj,
        movel,
        movej,
        set_tool,
        set_tcp,   
        amovel,
        DR_AXIS_Y
    )    
    
    def grip():
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        
    def release():
        set_digital_output(1, OFF)
        set_digital_output(2, ON)     
    
    # 변수 설정
    Jready = [0, 0, 90, 0, 90, 0]
    cup_pos= [422.62, -226.39, 260 , 144.44 , -179.8 , 143.76]
    target_pos=[357.28, -70.43, 200, 44.48, -179.81, 40.20]    
    set_tool("Tool Weight")
    set_tcp("GripperDA_v1")
    
    # 컵 쌓기 변수 설정
    cnt = 0
    total_layer = 3
    current_layer = 0
    movej(Jready, vel=50, acc=50)
    amovel(pos=posx([0,0,-100,0,0,0]), vel=100, acc=100, 
           ref=DR_TOOL, mod=DR_MV_MOD_REL, radius=10) 
       
            
    for i in range(total_layer):
        start_pos = target_pos[:]
        start_pos[0] += (40*i)
        start_pos[1] += (40*i)
        start_pos[2] += (95*i)
        current_line = 0
        
        for j in range(total_layer-current_layer):
            pose = start_pos[:]
            pose[0] += (40*j)
            pose[1] += (75*j)
            
            for k in range((total_layer - current_layer) - current_line):
                grip()
                
                # 컵 잡기
                if i == 0 and cnt != 0:
                    movel(pos=posx([0,0,-150,0,0,0]), vel=100, acc=100, ref=DR_TOOL, mod=DR_MV_MOD_REL) 
                    movel(cup_pos, vel=100, acc=100, ref=DR_BASE, mod=DR_MV_MOD_ABS) 
                else:
                    movel(cup_pos, vel=100, acc=100, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                    
                movel(pos=posx([0, 0, 35 + 12*cnt, 0, 0, 0]), vel=100, acc=100, ref=DR_TOOL, mod=DR_MV_MOD_REL)

                task_compliance_ctrl()
                set_desired_force(fd=[0,0,-10,0,0,0], dir=[0,0,1,0,0,0], mod=DR_FC_MOD_REL)

                while not check_force_condition(DR_AXIS_Z, max=5):
                    pass
                
                release_compliance_ctrl()
                
                movel(pos=posx([0,0,-10,0,0,0]), vel=100, acc=100, ref=DR_TOOL, mod=DR_MV_MOD_REL)
                release()
                movel(pos=posx([0,0,25,0,0,0]), vel=100, acc=100, ref=DR_TOOL, mod=DR_MV_MOD_REL)
                grip()
                movel(pos=posx([0,0,-150,0,0,0]), vel=100, acc=100, ref=DR_TOOL, mod=DR_MV_MOD_REL)
                
                # 컵 쌓기
                go_pose = pose[:]
                go_pose[0] += 80*k
                movel(go_pose, vel=100, acc=100, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                movel(pos=posx([0,0,115,0,0,0]), vel=100, acc=100, 
                      ref=DR_TOOL, mod=DR_MV_MOD_REL)
                    
                task_compliance_ctrl()
                set_desired_force(fd=[0,0,-10,0,0,0], dir=[0,0,1,0,0,0], mod=DR_FC_MOD_REL)
            
                while not check_force_condition(DR_AXIS_Z, max=5):
                    pass
            
                release_compliance_ctrl()      
                release()
                
                movel(pos=posx([0,0,-40, 0, 0, 0]), vel=100, acc=100, ref=DR_TOOL, mod=DR_MV_MOD_REL)
                cnt += 1  
                
                
            current_line += 1
        current_layer += 1
    
    # 컵 뒤집어서 쌓기    
    pose = [-70.26, 9.26, 107.65, 84.54, 100.38, -27.38]
    side_pose = [439.81, -216.13, 76.57, 35.65, 89.4, 90.13]  
          
    movel(pos=posx([0,0,-30,0,0,0]), vel=100, acc=100, ref=DR_TOOL, mod=DR_MV_MOD_REL)
    movel(cup_pos, vel=100, acc=100, ref=DR_BASE, mod=DR_MV_MOD_ABS)
    movej(pose, vel=50, acc=50)
    release()
     
    movel(side_pose, vel=100, acc=100, ref=DR_BASE, mod=DR_MV_MOD_ABS)
    grip()
    
    amovel(pos=posx([0,0,250,0,0,0]),vel=100, acc=100, ref=DR_BASE, mod=DR_MV_MOD_REL)
    movel(pos=posx([0,0,0,0,0,180]),vel=100, acc=100, ref=DR_TOOL, mod=DR_MV_MOD_REL)
    movel(pos=posx([7, 240, 0, 0, 0 ,0]), vel=100, acc=100, ref=DR_BASE, mod=DR_MV_MOD_REL)
    
    task_compliance_ctrl()
    set_desired_force(fd=[0,0,-10,0,0,0], dir=[0,0,1,0,0,0], mod=DR_FC_MOD_REL)
       
    while not check_force_condition(DR_AXIS_Z, max=5):
        pass
                
    release_compliance_ctrl()
    release()

    movel(pos=posx([0,0,-100,0,0,0]),vel=100, acc=100, ref=DR_TOOL, mod=DR_MV_MOD_REL)
    
    movej(pos=posj([-48.35, -1.26, 91.59, 0.17, 90.08, -47.35]), vel=100, acc=100)
    grip()

    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
