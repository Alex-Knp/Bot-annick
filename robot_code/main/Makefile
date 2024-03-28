#/*
# * Copyright (C) 2014  RoboPeak
# * Copyright (C) 2014 - 2018 Shanghai Slamtec Co., Ltd.
# *
# * This program is free software: you can redistribute it and/or modify
# * it under the terms of the GNU General Public License as published by
# * the Free Software Foundation, either version 3 of the License, or
# * (at your option) any later version.
# *
# * This program is distributed in the hope that it will be useful,
# * but WITHOUT ANY WARRANTY; without even the implied warranty of
# * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# * GNU General Public License for more details.
# *
# * You should have received a copy of the GNU General Public License
# * along with this program.  If not, see <http://www.gnu.org/licenses/>.
# *
# */
#
HOME_TREE := ../

MODULE_NAME := $(notdir $(CURDIR))

include $(HOME_TREE)/mak_def.inc

<<<<<<< HEAD
<<<<<<< HEAD
CXXSRC +=  ../localization/localisation.cpp main.cpp all_struct.cpp ../communication/SPI_spidev.cpp ../Path_planning/Path_planning.cpp ../Sensors/Micro_Switch.cpp ../Sensors/odometry.cpp 
=======
CXXSRC += ../localization/localisation.cpp main.cpp all_struct.cpp ../Path_planning/Path_planning.cpp 
>>>>>>> 2a719f0 (a)
=======
CXXSRC += ../localization/localisation.cpp main.cpp all_struct.cpp ../Path_planning/Path_planning.cpp motor_ask.cpp
>>>>>>> 8cdd76b (path)
C_INCLUDES += -I$(CURDIR)/../sdk/include -I$(CURDIR)/../sdk/src  #-L./home/student/lidar_example_cpp-main/output/Linux/Release/

EXTRA_OBJ :=
LD_LIBS += -lstdc++ -lpthread -lm -pthread

all: build_app

include $(HOME_TREE)/mak_common.inc

clean: clean_app