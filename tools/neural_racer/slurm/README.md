# Training the neural racer on FIR

> **Who this is for:** whoever submits neural-racer training jobs to Fir.
> **Read first:** [the neural racer plan](../../../docs/superpowers/specs/2026-10-23-neural-racer-plan.md#running-it-on-fir-at-over-80-gpu-efficiency) — why the jobs are shaped this way.
> **You'll be able to:** set up once, benchmark, train one stage, run the league, and check each job's GPU efficiency.

These scripts run the plan's training on the Alliance's **Fir** cluster at SFU. Each job uses one H100. The scripts are built to keep that GPU over 80% busy, and to put the CPU cores it leaves idle to work.

> **Status, 2026-10-23:** the scripts are tested locally against a stub trainer: signal forwarding, checkpoint-and-resume, the DONE and FAILED markers, and the utilisation summary. They have **not been run on Fir yet**.
>
> The trainers they launch (`neural_racer_train/<stage>.py`) don't exist until milestones M1 to M6. Until then, a job exits within seconds with exit code 3 and says so.

| File | What it does |
|---|---|
| `fir_train.sbatch` | one learner on one GPU: any stage (`teacher`, `distill`, `league_learner`, `benchmark`) or league role. Resumable |
| `fir_league_submit.sh` | submits all four league roles, each as a chain of resumable 12-hour segments |
| `fir_env.sh` | sourced by each job: loads modules and builds the venv on the node's local disk |
| `../requirements-fir.txt` | what that venv contains |

## One-time setup, on a Fir login node

Login nodes have internet access; compute nodes don't. So anything that downloads happens here, once.

**Terminal 1** — a Fir login node, from your clone of this workspace:

```bash
module load StdEnv/2023 python/3.11
tools/f1tenth_sim/setup.sh          # clones the pinned F1TENTH Gym into .sim/ for the CPU referee
avail_wheels torch numpy scipy PyYAML tensorboard   # confirm the wheelhouse carries them
```

**Working when:** `setup.sh` finishes without errors, and `avail_wheels` lists a version for each package.

**If it doesn't:** if a package is missing from the wheelhouse, download it into a directory of your own. Then pass that directory to jobs as `NR_WHEELS`:

```bash
pip download --no-deps -d "$PROJECT/nr-wheels" <package>
```

## Running

**Run these in order.** Each command is for a Fir login node, from the workspace root. Put your allocation's account name in place of `def-<pi>`.

**1. Measure throughput first.** Run this whenever the simulator or the network changes. The `benchmark` stage finds the environment count that fills the GPU.

```bash
sbatch --account=def-<pi> --time=00:30:00 tools/neural_racer/slurm/fir_train.sbatch benchmark bench-$(date +%F)
```

**Working when:** `$SCRATCH/neural_racer/runs/bench-<date>/solo/` contains the benchmark's report, and `gpu_util_summary_<job>.txt` reads 80% or more.

**2. A single stage**, for example Stage 1's teacher, as one seed:

```bash
sbatch --account=def-<pi> --export=ALL,NR_ROLE=seed0 \
    tools/neural_racer/slurm/fir_train.sbatch teacher teacher-v1
```

Resubmit the same command to continue it. It resumes from `$SCRATCH/neural_racer/runs/teacher-v1/seed0/checkpoints/`.

**3. The league:**

```bash
tools/neural_racer/slurm/fir_league_submit.sh --account def-<pi> --run league-v1 \
    --init "$SCRATCH/neural_racer/runs/distill-v1/solo/checkpoints/latest" --segments 6
```

Add `--dry-run` to print the `sbatch` commands without submitting anything.

**Working when:** `squeue -u $USER` shows four running jobs (one per role) and twenty pending ones, waiting on dependencies.

## Reading the results

Each role writes to `$SCRATCH/neural_racer/runs/<run>/<role>/`. All roles share `<run>/archive/`, so every role's snapshots become opponents for all the others.

| File | Meaning |
|---|---|
| `gpu_util_<job>.csv` | `nvidia-smi` sampled every 30 s |
| `gpu_util_summary_<job>.txt` | mean GPU utilisation for the whole job (what the Alliance sees) and after the first 10 minutes. It warns below 80% |
| `eval_<job>.log` | the CPU referee workers |
| `DONE` | the run used its whole budget. Later segments exit at once |
| `FAILED` | the trainer crashed. Later segments refuse to start until you fix the cause and delete this file |

After any job, `seff <jobid>` shows the scheduler's own CPU and memory efficiency.

## Why the job is shaped like this

- **One GPU, 12 cores, 64 GB.** One GPU's share of a Fir node is 12 cores and about 250 GB. Asking for more than the share is billed as more GPUs, and waits longer to start.
  - The network is too small for multi-GPU data parallelism to pay off. More GPUs means more league roles or seeds, each its own job.
- **Spare cores do real work.** The trainer keeps the simulator on the GPU, so it needs about one core. Of the other ten, three generate fresh tracks for the GPU's track pool (`NR_TRACKGEN_WORKERS`), and seven run the CPU referee (`tools/f1tenth_sim`) on archive snapshots. Without them, CPU efficiency would read about 8%.
- **12-hour segments, chained with `afterany`.** Shorter jobs start sooner. Slurm sends `SIGUSR1` 10 minutes before the limit; the script forwards it, and the trainer checkpoints and exits with code 85, which the script treats as success. So a pre-empted or timed-out segment never loses more than one checkpoint interval.

## The trainer contract

What every `neural_racer_train/<stage>.py` must do, so that these scripts work with it unchanged:

- Accept `--run-dir DIR --archive DIR --role NAME --cpu-workers N --resume`, plus its own options, and start N CPU track-generator processes (Phase 4 of the plan).
- With `--resume`, continue from `DIR/checkpoints/latest` if it exists.
- On `SIGUSR1`, write a checkpoint and exit **85**.
- Exit **0** only when the run's whole budget is done. Any other code is a crash.
- Never call `.item()` or otherwise synchronise with the GPU inside a rollout. Log every N updates. This is the single most common reason a GPU-resident trainer falls below 80%.

> **TODO(verify) before the first real submission:** the Fir wiki still listed access details as "to be determined" on 2026-10-23. Confirm the GPU request syntax (`--gpus-per-node=h100:1`), the maximum wall time, and whether the Python module is `python/3.11` or newer.
