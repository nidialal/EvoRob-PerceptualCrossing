#!/bin/bash -e

#SBATCH --job-name testNidia
#SBATCH --time=00:01:00     # Walltime
#SBATCH -A uoa00487         # Project Account
#SBATCH --mem-per-cpu=1G
#SBATCH --array=1-10%50   # Array: 1, 2, ..., 999, 1000.
                            # No more than 50 array tasks may be running at
                            # any time.
                            # On Pan, we have capped the number of array
                            # indices per array job to 1,000.


# In this example, the array index is being passed into array_binary
# as an argument.

python testSlurm.py $SLURM_ARRAY_TASK_ID