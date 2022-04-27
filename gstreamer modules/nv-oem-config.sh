#!/bin/bash

# Copyright (c) 2019-2021, NVIDIA CORPORATION.  All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#  * Neither the name of NVIDIA CORPORATION nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
# OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

DISPLAY_PLUGGED="$(cat /sys/class/graphics/fb*/device/panel_connected)"
KERNEL_CMDLINE="$(cat /proc/cmdline)"
/bin/systemctl start nv-l4t-usb-device-mode.service

if [[ -f /usr/lib/nvidia/resizefs/nvresizefs.templates ]]; then
	/usr/bin/debconf-loadtemplate nvresizefs \
		/usr/lib/nvidia/resizefs/nvresizefs.templates
fi

if [[ -f /usr/lib/nvidia/swap/nvswap.templates ]]; then
	/usr/bin/debconf-loadtemplate nvswap \
		/usr/lib/nvidia/swap/nvswap.templates
fi

if [[ -f /usr/lib/nvidia/qspi-update/nvqspi-update.templates ]]; then
	/usr/bin/debconf-loadtemplate nvqspi-update \
		/usr/lib/nvidia/qspi-update/nvqspi-update.templates
fi

if [[ -f /usr/lib/nvidia/nvpmodel/nvpmodel.templates ]]; then
	/usr/bin/debconf-loadtemplate nvpmodel \
		/usr/lib/nvidia/nvpmodel/nvpmodel.templates
fi

# Wait for the process to finish which has aquired this lock
while fuser "/var/cache/debconf/config.dat" > "/dev/null" 2>&1; do sleep 1; done;
while fuser "/var/cache/debconf/templates.dat" > "/dev/null" 2>&1; do sleep 1; done;

NV_AUTO_CONFIG=false
if [[ "${KERNEL_CMDLINE}" =~ "nv-auto-config" ]] && [[ -e "/nv_preseed.cfg" ]]; then
	# check preseed file format
	if /usr/bin/debconf-set-selections -c /nv_preseed.cfg; then
		while ! /usr/bin/debconf-set-selections /nv_preseed.cfg; do sleep 1; done;

		# oem-config auto mode only supports standard ubiquity plugins,
		# so remove NV plugins before running oem-config auto mode.
		rm -f /usr/lib/ubiquity/plugins/nv*.py

		touch /etc/nv/nvautoconfig
		NV_AUTO_CONFIG=true
	fi
fi

if [[ "${NV_AUTO_CONFIG}" = true ]] || [[ "${DISPLAY_PLUGGED}" =~ "1" ]]; then
	if [[ "${NV_AUTO_CONFIG}" = true ]]; then
		/bin/echo "Please wait for auto system configuration setup to complete..." > "/dev/kmsg"
	else
		/bin/echo "Please complete system configuration setup on desktop to proceed..." > "/dev/kmsg"
	fi
	/bin/systemctl start nv-oem-config-gui.service
else
	CONF_FILE="/etc/nv-oem-config.conf"

	UART_PORT="$(grep uart-port "${CONF_FILE}" | cut -d '=' -f 2)"
	if [[ -n "${UART_PORT}" ]]; then
		for i in {1..5}; do
			if [[ -e "/dev/${UART_PORT}" ]]; then
				break;
			elif [[ "${i}" =~ "5" ]]; then
				if [[ -e "/dev/ttyTCU0" ]]; then
					UART_PORT="ttyTCU0"
				elif [[ -e "/dev/ttyS0" ]]; then
					UART_PORT="ttyS0"
				else
					UART_PORT=""
				fi
			else
				sleep 1
			fi
		done
	fi

	if [[ -n "${UART_PORT}" ]]; then
		msg="Please complete system configuration setup on the serial port "
		msg+="provided by Jetson's USB device mode connection. e.g. "

		if [[ "${UART_PORT}" =~ "ttyGS0" ]]; then
			msg+="/dev/ttyACMx "
		else
			msg+="/dev/ttyUSBx "
		fi

		msg+="where x can 0, 1, 2 etc."

		/bin/echo "${msg}" > "/dev/kmsg"
		/bin/stty -F "/dev/${UART_PORT}" 115200 cs8 -parenb -cstopb
		/bin/systemctl start "nv-oem-config-debconf@${UART_PORT}.service"
	else
		/bin/echo "No serial port found to configure system!" > "/dev/kmsg"
	fi
fi
exit 0
