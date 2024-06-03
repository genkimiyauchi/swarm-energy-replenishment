#!/bin/bash

# Get first parameter

EXPERIMENT_NAME=work_and_charge
STRATEGY_TYPE="$1"

if [ "$STRATEGY_TYPE" = "fixed" ]; then
    STRATEGY_NAME=fixed_charger
    LINE_SEED=11
    LINE_TRIAL_DIR=99
    LINE_CHARGER_FULL_CAPACITY=115
    LINE_CHARGER_START_CAPACITY=116
    LINE_IDLE_ENERGY=117
    LINE_WORK_ENERGY=121
    LINE_RECHARGE_PER_STEP=122
    LINE_TRANSFER_LOSS=123
    LINE_NUM_CHARGERS=129
    NUM_CHARGERS=(0)
    CHARGER_CAPACITY=(0)

    # work rate
    # DELTA_WORK=(0.1 0.25 0.5 1.0 1.5 2.0 3.0 4.0 10.0)
    DELTA_WORK=(1.0)

    # recharge rate
    # DELTA_RECHARGE=(0.1 0.25 0.5 0.75 1.0)
    DELTA_RECHARGE=(1.0)

    # transfer loss
    DELTA_TRANSFER_LOSS=(0.0)

elif [ "$STRATEGY_TYPE" = "mobile" ]; then
    STRATEGY_NAME=mobile_charger
    LINE_SEED=11
    LINE_TRIAL_DIR=99
    LINE_CHARGER_FULL_CAPACITY=115
    LINE_CHARGER_START_CAPACITY=116
    LINE_IDLE_ENERGY=117
    LINE_WORK_ENERGY=121
    LINE_RECHARGE_PER_STEP=122
    LINE_TRANSFER_LOSS=123
    LINE_NUM_CHARGERS=129

    # number of chargers
    # NUM_CHARGERS=(1 2 4 6 8 10 12)
    NUM_CHARGERS=(1)

    # charger capacity
    # CHARGER_CAPACITY=(100 150 200 400 600)
    CHARGER_CAPACITY=(100)

    # work rate
    # DELTA_WORK=(0.1 0.25 0.5 1.0 1.5 2.0 3.0 4.0 10.0)
    DELTA_WORK=(1.0)

    # recharge rate
    # DELTA_RECHARGE=(0.1 0.25 0.5 0.75 1.0)
    DELTA_RECHARGE=(1.0)

    # transfer loss
    # DELTA_TRANSFER_LOSS=(0.0 0.1 0.2 0.3 0.4 0.5)
    DELTA_TRANSFER_LOSS=(0.0)
else
    echo "Invalid strategy type: '$1'"
    exit 1
fi

# ACTION NEEDED: Replace '/path/to/folder' to the path to the experiment directory
EXPERIMENT_DIR=/path/to/folder/swarm-energy-replenishment_prep/experiments/$EXPERIMENT_NAME
RESULT_DIR=/path/to/folder/swarm-energy-replenishment_prep/results/$EXPERIMENT_NAME
TEMPLATE_EXPERIMENT_FILE=$EXPERIMENT_DIR/${STRATEGY_NAME}.argos

# DEFAULT_DELTA_IDLE=0.0
DEFAULT_DELTA_IDLE=0.005
DEFAULT_DELTA_WORK=0.1
DEFAULT_DELTA_CHARGE=1.0
DEFAULT_DELTA_TRANSFER_LOSS=0.0

count=0
print_count=""

# Output color
ORANGE='\033[38;5;208m'
YELLOW='\033[0;33m'
NC='\033[0m' # No Color

# Start timing
start=$(date +%s.%N)
date +%r

### RUN multiple experiments

for o in ${DELTA_TRANSFER_LOSS[@]} # Transfer loss
do
    for m in ${DELTA_RECHARGE[@]} # Recharge per step
    do
        for j in ${DELTA_WORK[@]} # Worker energy
        do
            for i in ${CHARGER_CAPACITY[@]} # Charger capacity
            do
                for l in ${NUM_CHARGERS[@]} # Number of chargers per team
                do
                    # Start timing
                    start_config=$(date +%s.%N)

                    seed_count=0

                    for k in {1..50} # Number of runs
                    do
                        # Start timing
                        start_run=$(date +%s.%N)
                        
                        ((count++))

                        # Set log directory number
                        printf -v print_count "%03d" $k
                        # Make trial directory
                        TRIAL_DIR="${STRATEGY_TYPE}_${j}W_${l}C_${i}E_${m}R_${o}L_$print_count"
                        TRIAL_DIR_PATH="$RESULT_DIR/$TRIAL_DIR"
                        # echo $TRIAL_DIR_PATH
                        mkdir -p $TRIAL_DIR_PATH

                        # copy template experiment file into TRIAL_DIR
                        EXPERIMENT_FILE="$TRIAL_DIR_PATH/${STRATEGY_NAME}.argos"
                        cp $TEMPLATE_EXPERIMENT_FILE $EXPERIMENT_FILE

                        # Set seed
                        let seed=876*k
                        sed -i "${LINE_SEED}s|.*|random_seed='$seed' />|" $EXPERIMENT_FILE

                        # # Set run number
                        # sed -i "${LINE_RUN_NUMBER}s/.*/run_number='$k'/" $EXPERIMENT_FILE
                        echo $TRIAL_DIR
                        echo $TRIAL_DIR_PATH

                        # Set trial directory path
                        sed -i "${LINE_TRIAL_DIR}s|.*|out_path='results/${EXPERIMENT_NAME}/${TRIAL_DIR}/'|" $EXPERIMENT_FILE

                        # Set idle energy
                        sed -i "${LINE_IDLE_ENERGY}s|.*|delta_time='$DEFAULT_DELTA_IDLE'|" $EXPERIMENT_FILE

                        # Set work energy
                        rate=$(echo "$DEFAULT_DELTA_WORK*$j" | bc -l)
                        rate=$(printf "%.3f" $rate)
                        sed -i "${LINE_WORK_ENERGY}s|.*|<extra_battery_info delta_work='$rate'|" $EXPERIMENT_FILE

                        # Set recharge per step
                        rate=$(echo "$DEFAULT_DELTA_CHARGE*$m" | bc -l)
                        rate=$(printf "%.3f" $rate)
                        sed -i "${LINE_RECHARGE_PER_STEP}s|.*|delta_recharge='$rate'|" $EXPERIMENT_FILE

                        # Set transfer loss
                        rate=$(echo "$DEFAULT_DELTA_TRANSFER_LOSS*$o" | bc -l)
                        rate=$(printf "%.3f" $rate)
                        sed -i "${LINE_TRANSFER_LOSS}s|.*|delta_transfer_loss='$o'/>|" $EXPERIMENT_FILE

                        if [ "$STRATEGY_TYPE" = "mobile" ]; then
                            # # Set charger move energy
                            # sed -i "${LINE_CHARGER_MOVE_ENERGY}s|.*|delta_pos_charger='$rate'/>|" $EXPERIMENT_FILE

                            # Set charger full capacity
                            sed -i "${LINE_CHARGER_FULL_CAPACITY}s|.*|full_charge_charger='$i'|" $EXPERIMENT_FILE

                            # Set charger start capacity
                            sed -i "${LINE_CHARGER_START_CAPACITY}s|.*|start_charge_charger='$i,$i'|" $EXPERIMENT_FILE

                            # Set number of chargers
                            sed -i "${LINE_NUM_CHARGERS}s|.*|<e-puck_charger controller='charger' num_robots='$l'/>|" $EXPERIMENT_FILE
                        fi

                        # Run experiment
                        echo "Running experiment $count, t_loss = $o, charge_rate = $m, work_rate = $j, num_chargers = $l, capacity = $i, run = $k, (s = $seed) ..."
                        # LOG_FILE="$RESULT_DIR/log.txt"
                        # LOG_ERR_FILE="$RESULT_DIR/log_err.txt"

                        # Set log files to be discarded when running argos3
                        LOG_FILE="/dev/null"
                        LOG_ERR_FILE="/dev/null"
                        argos3 -c $EXPERIMENT_FILE -l $LOG_FILE -e $LOG_ERR_FILE --no-visualization
                        # argos3 -c $EXPERIMENT_FILE --no-visualization
                        # argos3 -c $EXPERIMENT_FILE

                        file="$TRIAL_DIR_PATH/summary.csv"
                        line1_num=8
                        line2_num=9
                        line3_num=10
                        line4_num=11
                        line1=$(sed "${line1_num}q;d" ${file})
                        line2=$(sed "${line2_num}q;d" ${file})
                        line3=$(sed "${line3_num}q;d" ${file})
                        line4=$(sed "${line4_num}q;d" ${file})
                        # Print the lines on the same line, with the first line in blue and the second line in green
                        echo -e "\033[32m${line1}, ${line2}, ${line3}, ${line4}\033[0m"

                        # End timing
                        end=$(date +%s.%N)

                        # Calculate the elapsed time in seconds
                        elapsed_seconds_run=$(echo "$end - $start_run" | bc)

                        # Convert the elapsed time to hours, minutes, and seconds
                        hours_run=$(printf "%02d" $(echo "$elapsed_seconds_run / 3600" | bc) 2>/dev/null)
                        minutes_run=$(printf "%02d" $(echo "($elapsed_seconds_run / 60) % 60" | bc) 2>/dev/null)
                        seconds_run=$(printf "%02d" $(echo "$elapsed_seconds_run % 60" | bc) 2>/dev/null)

                        # Print the elapsed time in hours, minutes, and seconds in orange color
                        echo -e "${YELLOW}RUN took: ${hours_run}:${minutes_run}:${seconds_run}${NC}"

                    done

                    # End timing
                    end=$(date +%s.%N)

                    # Calculate the elapsed time in seconds
                    elapsed_seconds_config=$(echo "$end - $start_config" | bc)
                    elapsed_seconds=$(echo "$end - $start" | bc)

                    # Convert the elapsed time to hours, minutes, and seconds
                    hours_config=$(printf "%02d" $(echo "$elapsed_seconds_config / 3600" | bc) 2>/dev/null)
                    minutes_config=$(printf "%02d" $(echo "($elapsed_seconds_config / 60) % 60" | bc) 2>/dev/null)
                    seconds_config=$(printf "%02d" $(echo "$elapsed_seconds_config % 60" | bc) 2>/dev/null)
                    hours=$(printf "%02d" $(echo "$elapsed_seconds / 3600" | bc) 2>/dev/null)
                    minutes=$(printf "%02d" $(echo "($elapsed_seconds / 60) % 60" | bc) 2>/dev/null)
                    seconds=$(printf "%02d" $(echo "$elapsed_seconds % 60" | bc) 2>/dev/null)

                    echo -e "${YELLOW}CONFIG took: ${hours_config}:${minutes_config}:${seconds_config}${NC}"
                    echo -e "${ORANGE}Elapsed time: ${hours}:${minutes}:${seconds}${NC}"
                    date +%r
                done
            done
        done
    done
done

### RUN one experiment

######

# # Parameters
# run_num=6
# print_count=""
# seed=943500
# team_num=4
# worker_per_team=9
# charger_per_team=1

# k="$run_num"
# i="$team_num"
# j="$worker_per_team"
# l="$charger_per_team"

# # Set seed
# sed -i "${LINE_SEED}s/.*/random_seed='$seed' \/>/" $EXPERIMENT_FILE

# # Set run number
# sed -i "${LINE_RUN_NUMBER}s/.*/run_number='$k'/" $EXPERIMENT_FILE

# # Set team size
# sed -i "${LINE_TEAM_SIZE}s/.*/<distribute_teams team_num='$i' robot_per_team='$j' charger_per_team='$l' center='0,0' density='0.2' worker_type='$WORKER_TYPE'\/>/" $EXPERIMENT_FILE

# # Set log directory number
# printf -v print_count "%03d" $k

# # Run experiment
# echo "Running experiment $count, No_teams = $i, robots_per_team = $j, chargers_per_team = $l, run = $k, (s = $seed) ..."
# LOG_FILE="$RESULT_DIR/log.txt"
# LOG_ERR_FILE="$RESULT_DIR/log_err.txt"

# # echo $LOG_FILE

# # Create folder if it doesn't exist
# if [ ! -d "${RESULT_DIR}" ]; then
#     mkdir -p "${RESULT_DIR}"
#     echo "Created folder: ${RESULT_DIR}"
# fi
# argos3 -c $EXPERIMENT_FILE -l $LOG_FILE -e $LOG_ERR_FILE
# # argos3 -c $EXPERIMENT_FILE
# NEW_LOG_FILE="$RESULT_DIR/${EXPERIMENT_NAME}(${STRATEGY_TYPE})_${i}T_${j}R_${l}C_$print_count/log.txt"
# NEW_LOG_ERR_FILE="$RESULT_DIR/${EXPERIMENT_NAME}(${STRATEGY_TYPE})_${i}T_${j}R_${l}C_$print_count/log_err.txt"
# # echo $LOG_FILE
# # echo $NEW_LOG_FILE
# mv $LOG_FILE $NEW_LOG_FILE
# mv $LOG_ERR_FILE $NEW_LOG_ERR_FILE
# # echo "done"

# file="$RESULT_DIR/${EXPERIMENT_NAME}(${STRATEGY_TYPE})_${i}T_${j}R_${l}C_$print_count/summary.csv"
# # Get the specified lines from the CSV file
# line1_num=9
# # line2_num=11
# line1=$(sed "${line1_num}q;d" ${file})
# # line2=$(sed "${line2_num}q;d" ${file})
# # Print the lines on the same line, with the first line in blue and the second line in green
# echo -e "\033[32m${line1}\033[0m"

# ######

# End timing
end=$(date +%s.%N)

# Calculate the elapsed time in seconds
elapsed_seconds=$(echo "$end - $start" | bc)

# Convert the elapsed time to hours, minutes, and seconds
hours=$(printf "%02d" $(echo "$elapsed_seconds / 3600" | bc) 2>/dev/null)
minutes=$(printf "%02d" $(echo "($elapsed_seconds / 60) % 60" | bc) 2>/dev/null)
seconds=$(printf "%02d" $(echo "$elapsed_seconds % 60" | bc) 2>/dev/null)

# Print the elapsed time in hours, minutes, and seconds in orange color
ORANGE='\033[38;5;208m'
NC='\033[0m' # No Color
echo -e "${ORANGE}TOTAL elapsed time: ${hours}:${minutes}:${seconds}${NC}"

date +%r