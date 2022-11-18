import transformations as tf
import numpy as np
import math
import struct


H_aeroRef_T265Ref = np.array([[1,0,0,0],[0,0,1,0],[0,-1,0,0],[0,0,0,1]])
H_T265body_aeroBody = np.linalg.inv(H_aeroRef_T265Ref)

def initData():
    global InitialAngle_flag
    global InitialAngle_Yaw
    InitialAngle_flag=False
    InitialAngle_Yaw=0


def solveData(pose_data):
    global InitialAngle_flag
    global InitialAngle_Yaw
    send_bufferOdometry=[0]*65
    qua0=pose_data.rotation.x
    qua1=pose_data.rotation.y
    qua2=pose_data.rotation.z
    qua3=pose_data.rotation.w
      
    H_T265Ref_T265body = tf.quaternion_matrix([pose_data.rotation.w, pose_data.rotation.x,pose_data.rotation.y,pose_data.rotation.z]) # in transformations, Quaternions w+ix+jy+kz are represented as [w, x, y, z]!
    # transform to aeronautic coordinates (body AND reference frame!)
    H_aeroRef_aeroBody = H_aeroRef_T265Ref.dot( H_T265Ref_T265body.dot( H_T265body_aeroBody ))
    rpy_rad = np.array( tf.euler_from_matrix(H_aeroRef_aeroBody, 'sxyz')) # Rz(yaw)*Ry(pitch)*Rx(roll) body w.r.t. reference frame

    if InitialAngle_flag == False :
        InitialAngle_Yaw = rpy_rad[2] * 180 /math.pi
        InitialAngle_flag = True
    yawerr = -( rpy_rad[2] * 180 /math.pi ) 
    position_x = pose_data.translation.z * math.cos( yawerr/180*math.pi) + pose_data.translation.x * math.sin(yawerr/180*math.pi)
    position_y = -pose_data.translation.z * math.sin(yawerr/180*math.pi) + pose_data.translation.x * math.cos(yawerr/180*math.pi)
    position_z = pose_data.translation.y

    velocity_x = pose_data.velocity.z * math.cos( yawerr/180*math.pi) + pose_data.velocity.x * math.sin(yawerr/180*math.pi)
    velocity_y = -pose_data.velocity.z * math.sin(yawerr/180*math.pi) + pose_data.velocity.x * math.cos(yawerr/180*math.pi)
    velocity_z = pose_data.velocity.y 
    
    send_bufferOdometry[0] = 0x55
    send_bufferOdometry[1] = 0xAA
    send_bufferOdometry[2] = 0x10
    
    position_x_buf = struct.pack("f",position_x)  
    send_bufferOdometry[3] = position_x_buf[0]
    send_bufferOdometry[4] = position_x_buf[1]
    send_bufferOdometry[5] = position_x_buf[2]
    send_bufferOdometry[6] = position_x_buf[3]

    position_y_buf = struct.pack("f",position_y)  
    send_bufferOdometry[7] = position_y_buf[0]
    send_bufferOdometry[8] = position_y_buf[1]
    send_bufferOdometry[9] = position_y_buf[2]
    send_bufferOdometry[10] = position_y_buf[3]

    position_z_buf = struct.pack("f",position_z) 
    send_bufferOdometry[11] = position_z_buf[0]
    send_bufferOdometry[12] = position_z_buf[1]
    send_bufferOdometry[13] = position_z_buf[2]
    send_bufferOdometry[14] = position_z_buf[3]

    velocity_x_buf = struct.pack("f",velocity_x) 
    send_bufferOdometry[15] = velocity_x_buf[0]
    send_bufferOdometry[16] = velocity_x_buf[1]
    send_bufferOdometry[17] = velocity_x_buf[2]
    send_bufferOdometry[18] = velocity_x_buf[3]

    velocity_y_buf = struct.pack("f",velocity_y) 
    send_bufferOdometry[19] = velocity_y_buf[0]
    send_bufferOdometry[20] = velocity_y_buf[1]
    send_bufferOdometry[21] = velocity_y_buf[2]
    send_bufferOdometry[22] = velocity_y_buf[3]

    velocity_z_buf = struct.pack("f",velocity_z)     
    send_bufferOdometry[23] = velocity_z_buf[0]
    send_bufferOdometry[24] = velocity_z_buf[1]
    send_bufferOdometry[25] = velocity_z_buf[2]
    send_bufferOdometry[26] = velocity_z_buf[3]

    Yaw_buf = struct.pack("f",rpy_rad[2]) 
    send_bufferOdometry[27] = Yaw_buf[0]
    send_bufferOdometry[28] = Yaw_buf[1]
    send_bufferOdometry[29] = Yaw_buf[2]
    send_bufferOdometry[30] = Yaw_buf[3]
 
    qua1_buf = struct.pack("f",qua1) 
    send_bufferOdometry[31] = qua1_buf[0]
    send_bufferOdometry[32] = qua1_buf[1]
    send_bufferOdometry[33] = qua1_buf[2]
    send_bufferOdometry[34] = qua1_buf[3]

    qua2_buf = struct.pack("f",qua2) 
    send_bufferOdometry[35] = qua2_buf[0]
    send_bufferOdometry[36] = qua2_buf[1]
    send_bufferOdometry[37] = qua2_buf[2]
    send_bufferOdometry[38] = qua2_buf[3]

    qua3_buf = struct.pack("f",qua3) 
    send_bufferOdometry[39] = qua3_buf[0]
    send_bufferOdometry[40] = qua3_buf[1]
    send_bufferOdometry[41] = qua3_buf[2]
    send_bufferOdometry[42] = qua3_buf[3]
    
#     print("\rrpy_rad[0]:{:.2f},rpy_rad[1]:{:.2f},rpy_rad[2]:{:.2f} ,X:{:.2f},Y:{:.2f},Z:{:.2f} ".format(rpy_rad[0]*57.3,rpy_rad[1]*57.3,rpy_rad[2]*57.3,pose_data.translation.x,pose_data.translation.y,pose_data.translation.z),end="")
    
    return send_bufferOdometry,position_x,position_y,position_z,rpy_rad