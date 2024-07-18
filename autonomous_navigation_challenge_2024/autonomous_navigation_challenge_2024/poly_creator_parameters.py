# flake8: noqa

# auto-generated DO NOT EDIT

from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.msg import FloatingPointRange, IntegerRange
from rclpy.clock import Clock
from rclpy.exceptions import InvalidParameterValueException
from rclpy.time import Time
import copy
import rclpy
from generate_parameter_library_py.python_validators import ParameterValidators



class poly_creator_parameters:

    class Params:
        # for detecting if the parameter struct has been updated
        stamp_ = Time()

        import_path = ""
        export_path = "./polygons.dat"



    class ParamListener:
        def __init__(self, node, prefix=""):
            self.prefix_ = prefix
            self.params_ = poly_creator_parameters.Params()
            self.node_ = node
            self.logger_ = rclpy.logging.get_logger("poly_creator_parameters." + prefix)

            self.declare_params()

            self.node_.add_on_set_parameters_callback(self.update)
            self.clock_ = Clock()

        def get_params(self):
            tmp = self.params_.stamp_
            self.params_.stamp_ = None
            paramCopy = copy.deepcopy(self.params_)
            paramCopy.stamp_ = tmp
            self.params_.stamp_ = tmp
            return paramCopy

        def is_old(self, other_param):
            return self.params_.stamp_ != other_param.stamp_

        def refresh_dynamic_parameters(self):
            updated_params = self.get_params()
            # TODO remove any destroyed dynamic parameters

            # declare any new dynamic parameters


        def update(self, parameters):
            updated_params = self.get_params()

            for param in parameters:
                if param.name == self.prefix_ + "import_path":
                    updated_params.import_path = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "export_path":
                    updated_params.export_path = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))



            updated_params.stamp_ = self.clock_.now()
            self.update_internal_params(updated_params)
            return SetParametersResult(successful=True)

        def update_internal_params(self, updated_params):
            self.params_ = updated_params

        def declare_params(self):
            updated_params = self.get_params()
            # declare all parameters and give default values to non-required ones
            if not self.node_.has_parameter(self.prefix_ + "import_path"):
                descriptor = ParameterDescriptor(description="The path to the file containing the data.", read_only = False)
                parameter = updated_params.import_path
                self.node_.declare_parameter(self.prefix_ + "import_path", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "export_path"):
                descriptor = ParameterDescriptor(description="The path to the file where the data is saved.", read_only = False)
                parameter = updated_params.export_path
                self.node_.declare_parameter(self.prefix_ + "export_path", parameter, descriptor)

            # TODO: need validation
            # get parameters and fill struct fields
            param = self.node_.get_parameter(self.prefix_ + "import_path")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.import_path = param.value
            param = self.node_.get_parameter(self.prefix_ + "export_path")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.export_path = param.value


            self.update_internal_params(updated_params)
