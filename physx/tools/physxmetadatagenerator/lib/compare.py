## SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
## SPDX-License-Identifier: Apache-2.0

# general utility module

import os
import sys
import re
from . import utils

def compareMetaDataDirectories(candidateDir, referenceDir):

	print("reference dir:", referenceDir)
	print("candidate dir:", candidateDir)

	if not _checkFileExistence(candidateDir, referenceDir):
		return False
	
	referenceFiles = utils.list_autogen_files(referenceDir)
	
	#get corresponding candidate files without relying on os.walk order
	def mapRefToCand(refFile):
		return os.path.join(candidateDir, os.path.relpath(refFile, referenceDir))
	candidateFiles = [mapRefToCand(f) for f in referenceFiles]
	
	for (fileCand, fileRef) in zip(candidateFiles, referenceFiles):
		
		timeCand = os.path.getmtime(fileCand)
		timeRef = os.path.getmtime(fileRef)
		
		if timeCand <= timeRef:
			print("last modified time of candidate is not later than last modified time of reference:")
			print("candidate:", fileCand, "\n", "reference:", fileRef)
			print("ref:", timeRef)
			print("cand:", timeCand)
			return False
		
		#_read_file_content will remove line endings(windows/unix), but not ignore empty lines
		candLines = _read_file_content(fileCand)
		refLines = _read_file_content(fileRef)
		if not (candLines and refLines):
			return False
				
		if len(candLines) != len(refLines):
			print("files got different number of lines:")
			print("candidate:", fileCand, "\n", "reference:", fileRef)
			print("ref:", len(refLines))
			print("cand:", len(candLines))
			return False

		for (i, (lineCand, lineRef)) in enumerate(zip(candLines, refLines)):
			if (lineCand != lineRef):
				print("candidate line is not equal to refence line:")
				print("candidate:", fileCand, "\n", "reference:", fileRef)
				print("@line number:", i)
				print("ref:", lineRef)
				print("cand:", lineCand)
				return False

	return True

##############################################################################
# internal functions
##############################################################################
	
#will remove line endings(windows/unix), but not ignore empty lines
def _read_file_content(filePath):
	lines = []
	try:
		with open(filePath, "r") as file:
			for line in file:
				lines.append(line.rstrip())
	except:
		print("issue with reading file:", filePath)
	
	return lines
	
def _checkFileExistence(candidateDir, referenceDir):
	candidateSet = set([os.path.relpath(f, candidateDir) for f in utils.list_autogen_files(candidateDir)])
	referenceSet = set([os.path.relpath(f, referenceDir) for f in utils.list_autogen_files(referenceDir)])
	
	missingSet = referenceSet - candidateSet
	if missingSet:
		print("the following files are missing from the candidates:\n", "\n".join(missingSet))
		return False
		
	excessSet = candidateSet - referenceSet
	if excessSet:
		print("too many candidate files:\n", "\n".join(excessSet))
		return False
	
	return True


