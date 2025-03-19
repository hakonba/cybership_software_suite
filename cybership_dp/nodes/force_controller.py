#!/usr/bin/env python3

import cybership_dp.CSAD.force_controller
import cybership_dp.voyager.force_controller
import rclpy
import rclpy.executors
import rclpy.node

import geometry_msgs.msg
import rcl_interfaces.msg

import os
import sys
import argparse

import numpy as np
import skadipy

import cybership_utilities.cybership_utilities.utilities
import cybership_utilities.cybership_utilities.utilities

import cybership_dp.voyager
import cybership_dp.enterprise
import cybership_dp.CSAD


class ForceControllerManager():

    VESSEL_MODEL_VOYAGER = cybership_utilities.utilities.VESSEL_MODEL_VOYAGER
    VESSEL_MODEL_ENTERPRISE = cybership_utilities.utilities.VESSEL_MODEL_ENTERPRISE
    VESSEL_MODEL_CSAD = cybership_utilities.utilities.VESSEL_MODEL_CSAD
    VESSEL_MODELS = [VESSEL_MODEL_VOYAGER, VESSEL_MODEL_ENTERPRISE, VESSEL_MODEL_CSAD]

    def __init__(self):

        self.vessel_model: str = None

        parser = argparse.ArgumentParser()
        parser.add_argument("--vessel-model", type=str, choices=ForceControllerManager.VESSEL_MODELS, required=True)
        self.args, _ = parser.parse_known_args()

    def initialize(self) -> rclpy.node.Node:

        if self.args.vessel_model == ForceControllerManager.VESSEL_MODEL_VOYAGER:
            return cybership_dp.voyager.force_controller.ForceControllerROS()
        
        elif self.args.vessel_model == ForceControllerManager.VESSEL_MODEL_VOYAGER:
            return cybership_dp.CSAD.force_controller.ForceControllerROS()

        elif self.args.vessel_model == ForceControllerManager.VESSEL_MODEL_ENTERPRISE:
            print("Force controller for C/S Enterprise is not implemented yet.")

            return None


def main(args=None):
    rclpy.init(args=args)

    manager = ForceControllerManager()

    node = manager.initialize()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
