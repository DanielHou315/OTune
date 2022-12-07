import os
import sys
import rosbag

import numpy as np



class CenterBackOptimizer:
    def __init__(self, bag_path):
        '''
        Construct a CenterBackOptimizer
        ----------------------------------------
        Variables we need for the optimzer are: 

        - Bag file path
        
        - Variables to Optimize
            - center wheel diameter
            - back wheel to center distance

        - Optimization Matrix
            - param_matrix              (Matrix M)
        
        - Input Matrices
            - wheel_angular_velocity    (Matrix A)
            - vive_velocity             (Matrix B)
        '''
        self.file_path = bag_path

        # Target Variables to Optimize
        self.center_wheel_diameter = 0.0
        self.back_wheel_distance = 0.0

        # Optimization vector
        self.param_matrix = np.array([[0.0], [0.0]])
        # Angular Velocity Matrix
        self.wheel_angular_velocity = []            # N*2 Matrix
        # Velocity measurements from vive
        self.vive_velocity = []                     # N*2 Vector


    def read_data(self, file_path):
        '''
        Read recorded odom wheel angular velocity data and the vive velocity data

        The Data in the ROS bag is formatted in the following way:
        - in topic 'vex_odom_wheel_velocity
            - vector.x is the angular velocity of the center wheel, in radps
            - vector.y is the angular velocity of the back wheel, in radps
        - in topic 'vive_velocity'
            - vector.x is linear velocity in the XY plane, in mps
                - It is calculated as a hypotenuse of X-linear-vel and Y-linear-vel to avoid the reference frame problem. 
            - vector.z is angular velocity around the Z-axis, in radps
        '''
        with rosbag.Bag(file_path, 'r') as bag:
            for topic, msg, t in bag.read_messages():
                if topic == 'vex_velocity':
                    self.wheel_angular_velocity.append([float(msg.x), float(msg.y)])
                elif topic == 'vive_velocity': 
                    self.vive_velocity.append([float(msg.x), float(msg.y)])
                    

    def calc_optimization(self):

        '''
        The optimization is based on the following function 

        M = (A.T * A).inv() * A.T * B
        
        where 
        - M is the target matrix 
        - A is the wheel angular velocity matrix 
        - B is the vive-measured velocity matrix

        The M Matrix takes the form
        --              --
        | D1,  0         |
        | 0 ,  D2/(2*B1) |
        --              --

        The A Matrix takes the form
        --         --
        | A11,  A12 |
        | A21,  A22 |
        | A31,  A32 |
        | ...,  ... |
        --         --

        The B Matrix takes the form
        --         --
        | B11,  B12 |
        | B21,  B22 |
        | B31,  B32 |
        | ...,  ... |
        --         --

        '''

        # Setup Least Square Model

        A = np.array(self.wheel_angular_velocity)
        AT = np.transpose(A)
        B = np.array(self.vive_velocity)
        #       X         =                                       (AT * A) ^-1 * AT * B
        M1 = np.matmul(AT, A)
        M2 = np.linalg.inv(M1)
        M3 = np.matmul(M2, AT)
        self.param_matrix = np.matmul(M3, B) 
       
        # Calculate the parameters  from the param_matrix
        # Detailed Derivation of this can be seen in attached optimization_derivation.pdf document
        self.center_wheel_diameter = (self.param_matrix[0][0] * 2) / 0.0254
        self.back_wheel_distance = 2.75 / self.param_matrix[1][1] / 2


    def show_optimization_results(self):
        '''
        Print and Log the results of the optimization
        '''
        print(self.param_matrix)
        print('''
        Parameters Optimized: 

        -----------------------------
        |                           |
        |            __             |
        |           |  |   D1       |
        |           |__|            |
        |       W1  ____            |
        |          |____|  D2       |
        |                           |
        |                           |
        -----------------------------

        Diameter of Center Tracking Wheel [D1]: {0} inches
        Diameter of Back Tracking Wheel [D1]: 2.75 inches
        Distance from Back Wheel to the center of robot: {1} inches

        '''.format(self.center_wheel_diameter, self.back_wheel_distance))


    def optimize(self, file_path):
        '''
        The function that should be called in the main process. 
        Conducts all steps of optimization and delivers a result
        '''
        # Read Data
        self.read_data(file_path)
        # Optimize
        self.calc_optimization()
        # Show Results
        self.show_optimization_results()
        



class LeftRightOptimizer:
    def __init__(self, bag_path):
        '''
        Construct a LeftRightOptimizer
        ----------------------------------------
        Variables we need for the optimzer are: 

        - Bag file path
        
        - Variables to Optimize
            - left wheel diameter
            - right wheel diameter
            - wheelbase

        - Optimization Matrix
            - param_matrix              (Matrix M)
        
        - Input Matrices
            - wheel_angular_velocity    (Matrix A)
            - vive_velocity             (Matrix B)
        '''
        self.file_path = bag_path

        # Target Variables to Optimize
        self.left_wheel_diameter = 0.0
        self.right_wheel_diameter = 0.0
        self.wheel_base = 0.0

        # Optimization vector
        self.param_matrix = np.array([[0.0], [0.0]])
        # Angular Velocity Matrix
        self.wheel_angular_velocity = []     # N*2 Matrix
        # Velocity measurements from vive
        self.vive_velocity = []              # N*2 Vector


    def read_data(self, file_path):

        '''
        Read recorded odom wheel angular velocity data and the vive velocity data

        The Data in the ROS bag is formatted in the following way:
        - in topic 'vex_odom_wheel_velocity
            - vector.x is the angular velocity of the left wheel, in radps
            - vector.y is the angular velocity of the right wheel,in radps
        - in topic 'vive_velocity'
            - vector.x is linear velocity in the XY plane, in mps
                - It is calculated as a hypotenuse of X-linear-vel and Y-linear-vel to avoid the reference frame problem. 
            - vector.z is angular velocity around the Z-axis, in radps
        '''
        with rosbag.Bag(file_path, 'r') as bag:
            for topic, msg, t in bag.read_messages():
                if topic == 'vex_velocity':
                    self.wheel_angular_velocity.append([float(msg.x), float(msg.y)])
                elif topic == 'vive_velocity': 
                    self.vive_velocity.append([float(msg.x), float(msg.y)])


    def calc_optimization(self):
        '''
        The optimization is based on the following function\

        M = (A.T * A).inv() * A.T * B
        
        where 
        - M is the target matrix 
        - A is the wheel angular velocity matrix 
        - B is the vive-measured velocity matrix

        The M Matrix takes the form
        --              --
        | D1/4      ,  D12/4     |
        | -D1/(4*B1),  D2/(4*B1) |
        --              --

        The A Matrix takes the form
        --         --
        | A11,  A12 |
        | A21,  A22 |
        | A31,  A32 |
        | ...,  ... |
        --         --

        The B Matrix takes the form
        --         --
        | B11,  B12 |
        | B21,  B22 |
        | B31,  B32 |
        | ...,  ... |
        --         --

        '''

        # Setup Least Square Model

        A = np.array(self.wheel_angular_velocity)
        AT = np.transpose(A)
        B = np.array(self.vive_velocity)
        #       X         =                                       (AT * A) ^-1 * AT * B
        M1 = np.matmul(AT, A)
        M2 = np.linalg.inv(M1)
        M3 = np.matmul(M2, AT)
        self.param_matrix = np.matmul(M3, B)

        # Calcualte Parameters from the param_matrix
        self.left_wheel_diameter = self.param_matrix[0][0] * 4 / 0.0254
        self.right_wheel_diameter = self.param_matrix[1][0] * 4 / 0.0254
        # self.wheel_base = (self.param_matrix[0][1] * -4 / self.left_wheel_diameter +  self.param_matrix[1][1] * 4 / self.right_wheel_diameter) / (2 * 0.0254)
        self.wheel_base = (self.param_matrix[0][1] * -1 / self.left_wheel_diameter +  self.param_matrix[1][1] / self.right_wheel_diameter) / 0.0254


    def show_optimization_results(self):
        '''
        Print and Log the results of the optimization
        '''
        print(self.param_matrix)

        print('''
        Parameters Optimized: 

        -----------------------------
        |                           |
        |      __          __       |
        | D1  |  |________|  |  D2  |
        |     |__|   W1   |__|      |
        |                           |
        |                           |
        -----------------------------

        Diameter of Left Tracking Wheel [D1]: {0}
        Diameter of Right Tracking Wheel [D2]: {1}
        Left of Wheel Base From Left to Right Wheel [W1]: {2}

        '''.format(self.left_wheel_diameter, self.right_wheel_diameter, self.wheel_base))


    def optimize(self, file_path):
        '''
        The function that should be called in the main process. 
        Conducts all steps of optimization and delivers a result
        '''
        # Read Data
        self.read_data(file_path)
        # Optimize
        self.calc_optimization()
        # Show Results
        self.show_optimization_results()



def auto_optimization(file_name):
    tuning_mode = -1

    os.system("rosbag reindex '"+ file_name + "'")

    with rosbag.Bag(file_name, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            if topic == 'tuning_mode' and tuning_mode < 0:
                tuning_mode = msg.data
    
    if tuning_mode == 1:
        optimizer = CenterBackOptimizer(file_name)
        optimizer.optimize(file_name)
    elif tuning_mode == 2:
        optimizer = LeftRightOptimizer(file_name)
        optimizer.optimize(file_name)



if __name__ == "__main__":
    tuning_mode = -1
    
    # Find File Name
    bag_name = ""
    if len(sys.argv) > 1:
        bag_name = sys.argv[1]

        if os.path.exists(bag_name) == False:
            print("!!! File name is invalid !!!")
        else:
            auto_optimization(bag_name)

    else:
        print('''Usage:

    - python3 otune_optimizer.py <BAG_FILE>
        
        ''')

    