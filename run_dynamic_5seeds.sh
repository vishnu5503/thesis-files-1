#!/bin/bash
# =====================================================================
# run_dynamic_5seeds.sh  —  5-seed runner for dynamic AQM comparison
# =====================================================================
# Flow composition (both controllers identical):
#   Base  : 120 BulkTCP + 60 OnOffTCP + 20 UDP = 200 flows
#   Extra :  30 BulkTCP + 15 OnOffTCP +  5 UDP =  50 flows per pool
#
# Dynamic events:
#   t=25s  : +50 flows (Extra-A)       => 250
#   t=40s  : qRef -> 300
#   t=50s  : +50 flows (Extra-B)       => 300
#   t=60s  : -60 flows (base BulkTCP)  => 240
#   t=75s  : qRef -> 100 (stress)
#   t=85s  : -50 flows (Extra-B ends)  => 190
#   t=95s  : qRef -> 700
#   t=110s : +50 flows (Extra-C)       => 240
# =====================================================================

NS3="$HOME/ns-3.45"
RESULTS="$HOME/results_dynamic"

echo "============================================================"
echo " Dynamic 5-Seed Experiment"
echo " PHAQM vs Hybrid MPC+PID+RLS"
echo " Base: 120 BulkTCP + 60 OnOffTCP + 20 UDP = 200"
echo " Extra pools: 30 BulkTCP + 15 OnOffTCP + 5 UDP = 50 each"
echo " qRef sequence: 500 -> 300 -> 100 -> 700"
echo "============================================================"
echo ""

cd "$NS3" || { echo "ERROR: $NS3 not found"; exit 1; }

echo "[BUILD] Building NS-3..."
./ns3 build 2>&1 | tail -3
echo ""

mkdir -p "$RESULTS/phaqm" "$RESULTS/mpc"

echo "[PHAQM] Running 5 seeds..."
for seed in 1 2 3 4 5; do
  OUTDIR="$RESULTS/phaqm/run_$seed"
  mkdir -p "$OUTDIR"
  echo -n "  seed=$seed ... "
  ./ns3 run "scratch/phaqm-dumbbell-dynamic \
    --seed=$seed --simTime=120 \
    --outDir=$OUTDIR" 2>&1 \
    | grep -E "Phase [0-9]|done\.|ERROR|assert" | tail -10
  echo "done -> $OUTDIR"
done

echo ""
echo "[MPC] Running 5 seeds..."
for seed in 1 2 3 4 5; do
  OUTDIR="$RESULTS/mpc/run_$seed"
  mkdir -p "$OUTDIR"
  echo -n "  seed=$seed ... "
  ./ns3 run "scratch/hybrid-mpc-dumbbell-dynamic \
    --seed=$seed --simTime=120 \
    --outDir=$OUTDIR" 2>&1 \
    | grep -E "Phase [0-9]|done\.|ERROR|assert" | tail -10
  echo "done -> $OUTDIR"
done

echo ""
echo "============================================================"
echo " ALL 10 RUNS COMPLETE"
echo ""
echo " Results:"
echo "   $RESULTS/phaqm/run_{1..5}/phaqm-queue-length.csv"
echo "   $RESULTS/phaqm/run_{1..5}/phaqm-phase-stats.csv"
echo "   $RESULTS/mpc/run_{1..5}/mpc-queue-length.csv"
echo "   $RESULTS/mpc/run_{1..5}/mpc-phase-stats.csv"
echo ""
echo " Generate plot:"
echo "   python3 ~/dynamic_comparison_plot.py"
echo "   eog ~/ns-3.45/dynamic_comparison.png"
echo "============================================================"
