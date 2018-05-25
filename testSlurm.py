from datetime import date as dt
import os.path as op
import os
import sys

num = sys.argv[1]

folderName = "Test_Slurm_" + str(num)

if not op.exists(folderName):
	os.makedirs(folderName)
