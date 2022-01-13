#include <spark_max_controllers/spark_max_controller.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(spark_max_controllers::SparkMaxDutyCycleController,
					   controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(spark_max_controllers::SparkMaxFollowerController,
					   controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(spark_max_controllers::SparkMaxPositionCloseLoopController,
					   controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(spark_max_controllers::SparkMaxVelocityCloseLoopController,
					   controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(spark_max_controllers::SparkMaxVoltageCloseLoopController,
					   controller_interface::ControllerBase)

