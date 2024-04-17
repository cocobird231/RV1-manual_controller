#ver=2.0
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import json
import yaml
from yaml import load, Loader

pkgName = 'cpp_controller'

def generate_launch_description():
    commonFilePath = os.path.join(get_package_share_directory(pkgName), 'launch/common.yaml')
    with open(commonFilePath, 'r') as f:
        data = yaml.load(f, Loader=Loader)

    serviceFilePath = os.path.join(get_package_share_directory(pkgName), 'launch/service.json')
    with open(serviceFilePath, 'r') as f:
        serviceData = json.load(f)

    return LaunchDescription([
        Node(
            package=pkgName,
            namespace=data['generic_prop']['namespace'],
            executable="pub",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "msg_type" : data['controller_prop']['msg_type'], 
                    "controller_mode" : data['controller_prop']['controller_mode'], 
                    "service_name" : data['controller_prop']['service_name'] + '_' + str(data['generic_prop']['id']), 
                    "timeout_ms" : data['controller_prop']['timeout_ms'], 
                    "period_ms" : data['controller_prop']['period_ms'], 
                    "privilege" : data['controller_prop']['privilege'], 
                    "pub_type" : data['controller_prop']['pub_type'], 

                    "mode" : data['input_prop']['mode'], 
                    "joystickConfigFile" : data['input_prop']['joystickConfigFile'], 
                    "keyboardConfigFile" : data['input_prop']['keyboardConfigFile'], 

                    "controlService" : data['controlserver_prop']['controlService'], 
                    "controllerRegService" : data['controlserver_prop']['controllerRegService'], 

                    # Settings for Params class under vehicle_interfaces/params.h
                    # Do not change the settings rashly
                    "nodeName" : data['generic_prop']['nodeName'] + '_' + str(data['generic_prop']['id']) + '_node', 
                    "id" : data['generic_prop']['id'], 
                    "devInfoService" : serviceData['devInfoService'], 
                    "devInterface" : serviceData['devInterface'], 
                    "devMultiNode" : serviceData['devMultiNode'], 
                    "qosService" : serviceData['qosService'], 
                    "qosDirPath" : serviceData['qosDirPath'], 
                    "safetyService" : serviceData['safetyService'], 
                    "timesyncService" : serviceData['timesyncService'], 
                    "timesyncPeriod_ms" : serviceData['timesyncPeriod_ms'], 
                    "timesyncAccuracy_ms" : serviceData['timesyncAccuracy_ms'], 
                    "timesyncWaitService" : serviceData['timesyncWaitService'], 
                }
            ]
        )
    ])