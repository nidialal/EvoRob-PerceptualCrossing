#!/bin/bash -e

#SBATCH -J NidiaPerceptualCrossing
#SBATCH --time=144:00:00     # Walltime
#SBATCH -A uoa00487         # Project Account
#SBATCH --mem-per-cpu=16G
#SBATCH --array=1-30   # Array: 1, 2, ..., 999, 1000.
                            # No more than 50 array tasks may be running at
                            # any time.
                            # On Pan, we have capped the number of array
                            # indices per array job to 1,000.

# In this example, the array index is being passed into array_binary
# as an argument.
module load Python/2.7.11-intel-2015a
python GA_robotPC_withHandRaising_fixedrob.py $SLURM_ARRAY_TASK_ID
