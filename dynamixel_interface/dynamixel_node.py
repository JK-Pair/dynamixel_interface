#!/usr/bin/env python3
# -*- coding: utf-8 -*-

try:
    from dynamixel_interface.dynamixel_sdk import *
    from dynamixel_interface.config_params import *
except:
    from dynamixel_sdk import *
    from config_params import *

import numpy as np

class dxl_communication():

    portHandler                 = PortHandler(DEVICENAME)
    packetHandler               = PacketHandler(PROTOCOL_VERSION)

    def __init__(self):
        #Here define all initial params using in this class.
        self.oneByteAddr = ADDR_TORQUE_ENABLE
        self.oneByteParam = PARAM_ENABLE

        # self.addrReadParam = ADDR_PRESENT_POSITION
        # self.lenReadParam = LEN_PRESENT_POSITION

        self.addrReadParam = ADDR_PRESENT_LOAD
        self.lenReadParam = LEN_PRESENT_LOAD

        self.addrWriteParam = ADDR_GOAL_POSITION
        self.lenWriteParam = LEN_GOAL_POSITION

        # Initialize Group Sync instace
        self.buildReadCommand()
        self.buildWriteCommand()

        # Adding motors id into the class
        self.motorIdManagement(TARGET_DXL)

    def buildWriteCommand(self):
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, self.addrWriteParam, self.lenWriteParam)

    def buildReadCommand(self):
        self.groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, self.addrReadParam, self.lenReadParam)

    def openPort(self):
        return self.portHandler.openPort() & self.portHandler.setBaudRate(BAUDRATE)
    
    def configReadingProtocol(self, address_list):
        '''
        This is a function to configure the desired address and calculate the data length 
        for reading data from the motor at that address.

        INPUT Param: 
        1. "Present Position" : For reading only Present Position
        2. ["Present Position", "Present Load", "Present Velocity"] : For reading data from all as once.

        OUTPUT Param:
        1. list of target address
        2. list of data length
        3. Start address for setting GroupSyncRead class
        '''
        self.address_return = []
        self.data_size_return = []

        if type(address_list) == str:
            self.address_return.append(ADDR_DICT[address_list][0])
            self.data_size_return.append(ADDR_DICT[address_list][1])

        elif type(address_list) == list:
            self.address_return.append(ADDR_DICT[address_list[0]][0])
            self.data_size_return.append(ADDR_DICT[address_list[0]][1])

            try:
                for addr in range(1, len(address_list)):
                    self.data_size_return.append(ADDR_DICT[address_list[addr]][1])
                    self.address_return.append(ADDR_DICT[address_list[addr]][0])

            except:
                pass

            #Start address
            self.addrReadParam = min(self.address_return)
            

        else:
            pass

        #The length of the data is a fixed number. Because the read address, load, speed, and position are next to each other.
        self.lenReadParam = 10 #sum(self.data_size_return)
        self.groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, self.addrReadParam, self.lenReadParam)
        self.buildReadCommand()

    def arrayManagement(self, input_list):
        '''
        INPUT PARAM: ["31","32","33", "11","12","13", "51","52","53"]
        OUTPUT PARAM:
                    [[31 32 33]
                    [11 12 13]
                    [51 52 53]]
        '''
        #Firstly, checking the motor id and target should be the same size
        # num_ids = self.num_motos
        #Reshape from the list to numpy array to better performance on calculation.

        num_legs = int(len(input_list)) #Each leg has 3 motors
        #print(num_legs)
        input_list_numpy =  np.array(input_list).reshape(num_legs, 1).astype('int')
        
        # #Finally, the motor id array would be return to the same array with int data type.
        return input_list_numpy

    def motorIdManagement(self, input_list):
        self.num_motos = len(input_list)
        self._arrangedId = self.arrayManagement(input_list)

    def packetHandlerGetError(self):

        if self.dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(self.dxl_comm_result))
        elif self.dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(self.dxl_error))
        else:
            print("Dynamixel#{} has been successfully [{}] connected".format(self.input_list, self.oneByteParam))
        

    def nestedSeqFunction(self, anyfunction, secondFunc=None):
        '''
        Define a vectorized function which takes a nested sequence of objects or numpy arrays 
        as inputs and returns a single numpy array or a tuple of numpy arrays.
        
        Here, passing _arrangedId one by one to anyFunction, which is a function received parameters to operate one thing.
        '''
        vector_func = np.vectorize(anyfunction, otypes=[list])
        # print(secondFunc)
        if(secondFunc is not None):
            #If we have more than 1 argument passing through the target function.
            read_result = vector_func(self._arrangedId, secondFunc)

        else:
            read_result = vector_func(self._arrangedId)

        return read_result
    
    def outputConversion2(self, read_result, data_type):
        #Convert from int to float
        numpy_2_list = read_result.flatten().tolist()
        print(numpy_2_list)
        convertDataInSide = list(map(data_type, numpy_2_list))
        return convertDataInSide
    
    def groupDataIntoNewForm(self, data_input):
        data_list = data_input.flatten().tolist()
        data_np_inversed = np.array(data_list).T

        return data_np_inversed
    
    def syncReadByte(self, input_list):

        #Updating address and size of target data for group sync read
        # self.buildReadCommand()
        # Add parameter storage before sending them as once
        dxl_addparam_result = self.groupSyncRead.addParam(input_list)

        if dxl_addparam_result != True:
            print("[ID:%03d] Sync_class addparam failed" % input_list)
            quit()

        # Syncread present position and LED status
        # dxl_comm_result = self.groupSyncRead.txRxPacket()
        dxl_comm_result = self.groupSyncRead.fastTxRxPacket()
            
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        # Get present position value
        # get_data = self.groupSyncRead.getData(input_list, self.addrReadParam, self.lenReadParam)
        data_packed_list = []

        for item in range(len(self.data_size_return)):
            get_data = self.groupSyncRead.getData(input_list, self.address_return[item], self.data_size_return[item])
            data_packed_list.append(self.negative_convert(get_data))

        self.groupSyncRead.clearParam()

        return data_packed_list

        # return get_data
    
    
    def negative_convert(self, targetData):

        #Checking wheter is negative or positive
        if (targetData >= (2**31)):
            targetData = targetData - (2**32)

        elif (targetData >= (2**15)):
            targetData = targetData - (2**16)

        return float(targetData)
    
    
    def syncWritePacket(self, id_array, input_data):
        #If the size of input data more than 2 byte, the sending packet would be changed.
        #Here, there are 2 byte and 4 byte

        #Converting every data type in term of number to only int.
        input_data = int(input_data)

        if (self.lenWriteParam == 1):

            data_to_byte = [input_data]

        elif(self.lenWriteParam == 2):
            data_to_byte = [
                                DXL_LOBYTE(DXL_LOWORD(input_data)), 
                                DXL_HIBYTE(DXL_LOWORD(input_data))
                            ]
            
        elif(self.lenWriteParam == 4):

            data_to_byte = [
                        DXL_LOBYTE(DXL_LOWORD(input_data)), 
                        DXL_HIBYTE(DXL_LOWORD(input_data)), 
                        DXL_LOBYTE(DXL_HIWORD(input_data)), 
                        DXL_HIBYTE(DXL_HIWORD(input_data))
                    ]
        else:
            print("Please check the data input again...")

            data_to_byte = [
                        DXL_LOBYTE(DXL_LOWORD(input_data)), 
                        DXL_HIBYTE(DXL_LOWORD(input_data)), 
                        DXL_LOBYTE(DXL_HIWORD(input_data)), 
                        DXL_HIBYTE(DXL_HIWORD(input_data))
                    ]
        # Add parameter storage for Dynamixel#1 present position
        dxl_addparam_result = self.groupSyncWrite.addParam(id_array, data_to_byte)
        return dxl_addparam_result

    def syncWriteByte(self):

        # Syncread goal position
        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        
        # Clear Syncwrite parameter storage
        self.groupSyncWrite.clearParam()

        return dxl_comm_result 
    

class dxl_interface(dxl_communication):

    def __init__(self):
        super().__init__()
        pass

    def torqueOperation(self, input_param):

        self.addrWriteParam, self.lenWriteParam = ADDR_TORQUE_ENABLE, LEN_ONE_BYTE
        super().buildWriteCommand()
        
        if(input_param):
            self.oneByteParam = PARAM_ENABLE
        else:
            self.oneByteParam = PARAM_DISABLE

        super().nestedSeqFunction(super().syncWritePacket, self.oneByteParam)
        super().syncWriteByte()

    def ledOperation(self, input_param):
        
        self.addrWriteParam, self.lenWriteParam = ADDR_LED_RED, LEN_ONE_BYTE
        super().buildWriteCommand()
        
        if(input_param):
            self.oneByteParam = PARAM_ENABLE
        else:
            self.oneByteParam = PARAM_DISABLE
            
        super().nestedSeqFunction(super().syncWritePacket, self.oneByteParam)
        super().syncWriteByte()

    def manualReadAddress(self, target_addr, size_data, data_type):
        self.addrReadParam, self.lenReadParam = target_addr, size_data
        response  = super().nestedSeqFunction(super().syncReadByte) 
        return self.outputConversion2(response, data_type)
    
    def readConnectedID(self):
        self.addrReadParam, self.lenReadParam = ADDR_ID_MOTOR, LEN_ONE_BYTE
        response  = super().nestedSeqFunction(super().syncReadByte) 
        return self.outputConversion2(response, str)
    
    def readDataFromDXL(self):
        # self.addrReadParam, self.lenReadParam = ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
        response = super().nestedSeqFunction(super().syncReadByte) 
        ans = self.groupDataIntoNewForm(response)
        return ans.tolist()
    
    def writeGoalPosition(self, goalPosition):
                
        self.addrWriteParam, self.lenWriteParam = ADDR_GOAL_POSITION, LEN_GOAL_POSITION
        super().buildWriteCommand()

        _goalPosition = super().arrayManagement(goalPosition)

        super().nestedSeqFunction(super().syncWritePacket, _goalPosition)
        super().syncWriteByte()
    
    def writeOperatingMode(self, targetMode):
                
        self.addrWriteParam, self.lenWriteParam = ADDR_OPERATING_MODE, LEN_ONE_BYTE
        super().buildWriteCommand()

        _goalPosition = super().arrayManagement(targetMode)

        super().nestedSeqFunction(super().syncWritePacket, _goalPosition)
        super().syncWriteByte()
        
    def writeGoalCurrent(self, targetCurrent):
                
        self.addrWriteParam, self.lenWriteParam = ADDR_GOAL_CURRENT, LEN_PRESENT_LOAD
        super().buildWriteCommand()

        _goalCurrent = super().arrayManagement(targetCurrent)

        super().nestedSeqFunction(super().syncWritePacket, _goalCurrent)
        super().syncWriteByte()

    def writeVelocityLimit(self, velocity_limit):
                
        self.addrWriteParam, self.lenWriteParam = ADDR_VELOCITY_LIMIT, LEN_PRESENT_POSITION
        super().buildWriteCommand()

        _vel_limit = super().arrayManagement(velocity_limit)

        super().nestedSeqFunction(super().syncWritePacket, _vel_limit)
        super().syncWriteByte()
        
    def writeProfileVelocity(self, profile_vel_limit):
                
        self.addrWriteParam, self.lenWriteParam = ADDR_PROFILE_VELOCITY, LEN_PRESENT_POSITION
        super().buildWriteCommand()

        _vel_limit = super().arrayManagement(profile_vel_limit)

        super().nestedSeqFunction(super().syncWritePacket, _vel_limit)
        super().syncWriteByte()

    def writeGoalVelocity(self, targetVelocity):
                
        self.addrWriteParam, self.lenWriteParam = ADDR_GOAL_VELOCITY, LEN_PRESENT_VELOCITY
        super().buildWriteCommand()

        _goalVelocity = super().arrayManagement(targetVelocity)

        super().nestedSeqFunction(super().syncWritePacket, _goalVelocity)
        super().syncWriteByte()

    def writeVelocityLimit(self, targetLimit):

        self.addrWriteParam, self.lenWriteParam = ADDR_VELOCITY_LIMIT, LEN_VELOCITY_LIMIT
        super().buildWriteCommand()

        
        _limitVelocity = super().arrayManagement(targetLimit)

        super().nestedSeqFunction(super().syncWritePacket, _limitVelocity)
        super().syncWriteByte()

if __name__ == "__main__":
    dxl_obj = dxl_interface()
    dxl_obj.openPort()
    # dxl_obj.arrayManagement(np.array([[31,32,33], [11,12,13]]))
    # dxl_obj.arrayManagement(np.array([["31","32","33"], ["11","12","13"], ["51","52","53"]]))
    dxl_obj.motorIdManagement(["1", "2", "3"])
    
    dxl_obj.configReadingProtocol(["Present Position", "Present Load", "Present Velocity"])
    dxl_obj.writeOperatingMode([0x01, 0x01, 0x05])
    dxl_obj.writeGoalVelocity([20, 20, 60])
    # print(dxl_obj.address_return)
    # print(dxl_obj.data_size_return)
    # print(dxl_obj.addrReadParam)
    print(dxl_obj.readDataFromDXL())

