#!/bin/bash

(
	set -x
	set -e
	
	this_script="$(readlink -e "${BASH_SOURCE[0]}")"
	this_script_dir="$(dirname "${this_script}")"
	
	dependency_file="${this_script_dir}/dependencies.txt"
	
	# dependency file should not have more than one newline at the end
	mapfile -t dependency_list < <(cat "${dependency_file}")
	for dep in "${dependency_list[@]}"; do
		arduino-cli lib uninstall "$dep"
	done
)
