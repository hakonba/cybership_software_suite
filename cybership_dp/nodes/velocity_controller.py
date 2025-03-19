#!/usr/bin/env python3

import cybership_dp.CSAD
import cybership_dp.CSAD.velocity_controller
import cybership_dp.voyager.force_controller
import cybership_dp.voyager.velocity_controller
import rclpy
import rclpy.executors
import rclpy.node

import geometry_msgs.msg
import rcl_interfaces.msg

import os
import sys
import argparse

import numpy as np

import cybership_utilities.cybership_utilities.utilities
import cybership_utilities.cybership_utilities.utilities

import cybership_dp.voyager
import cybership_dp.enterprise


class VelocityControllerManager():

    VESSEL_MODEL_VOYAGER = cybership_utilities.cybership_utilities.utilities.VESSEL_MODEL_VOYAGER
    VESSEL_MODEL_ENTERPRISE = cybership_utilities.cybership_utilities.utilities.VESSEL_MODEL_ENTERPRISE
    VESSEL_MODEL_CSAD = cybership_utilities.cybership_utilities.utilities.VESSEL_MODEL_CSAD
    VESSEL_MODELS = [VESSEL_MODEL_VOYAGER, VESSEL_MODEL_ENTERPRISE, VESSEL_MODEL_CSAD]

    def __init__(self):

        self.vessel_model: str = None

        parser = argparse.ArgumentParser()
        parser.add_argument("--vessel-model", type=str, choices=VelocityControllerManager.VESSEL_MODELS, required=True)
        self.args, _ = parser.parse_known_args()

    def initialize(self) -> rclpy.node.Node:

        if self.args.vessel_model == VelocityControllerManager.VESSEL_MODEL_VOYAGER:
            return cybership_dp.voyager.velocity_controller.VelocityControllerROS()
        
        elif self.args.vessel_model == VelocityControllerManager.VESSEL_MODEL_CSAD:
            return cybership_dp.CSAD.velocity_controller.VelocityControllerROS()

        elif self.args.vessel_model == VelocityControllerManager.VESSEL_MODEL_ENTERPRISE:
            print("Force controller for C/S Enterprise is not implemented yet.")

            return None


def main(args=None):
    rclpy.init(args=args)

    manager = VelocityControllerManager()

    node = manager.initialize()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
