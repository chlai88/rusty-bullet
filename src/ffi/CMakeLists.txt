# ***** BEGIN GPL LICENSE BLOCK *****
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software Foundation,
# Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
# The Original Code is Copyright (C) 2006, Blender Foundation
# All rights reserved.
#
# The Original Code is: all of this file.
#
# ***** END GPL LICENSE BLOCK *****
cmake_minimum_required(VERSION 3.0)

project(Rb_Bullet
	VERSION 0.1
)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../../../lib)

set(BULLET_BASE_DIR /Users/chlai/projects/bullet/bullet3)

set(BULLET_INCLUDE_DIRS 
	${BULLET_BASE_DIR}/src
)

set(BULLET_LIBRARIES 
	${BULLET_BASE_DIR}/lib/libBulletCollision_xcode4.a
	${BULLET_BASE_DIR}/lib/libBulletDynamics_xcode4.a
	${BULLET_BASE_DIR}/lib/libLinearMath_xcode4.a	
)

include_directories(
	${BULLET_INCLUDE_DIRS}
	.
)

set(SRC
	rb_bullet_api.cpp
)

add_library(rb_bullet STATIC ${SRC})

target_link_libraries(rb_bullet ${BULLET_LIBRARIES})
