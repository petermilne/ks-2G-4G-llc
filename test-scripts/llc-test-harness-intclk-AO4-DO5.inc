#!/bin/bash
# llc-test-harness-intclk-AO4-DO5 
# AI+AO llc with zerocopy on one fiber. Basic LLC test


# MUST set remote ip-address
OUTBOX=pizza2
echo "OUTBOX $OUTBOX"
INTCLKDIV=${INTCLKDIV:-1000}		# 100M/100 = 1MHz
EXTCLKDIV=${EXTCLKDIV:-4}		# 1M/4 = 250kHz
STREAMPID=0
LLC_TRG=${LLC_TRG:-ext}
LLC_CLK=${LLC_CLK:-ext}

# site definitions SC: system controller, CB: commsB, AI: AI, AO : AO
# AI uses commsA, AO uses commsB
SC=0
AICX=13
CB=12
AI=1
AO=4
DO=5

source ./test-scripts/afhba-common

in_cmd() {
	REMIP=$INBOX remote_cmd $*
}

out_cmd() {
	REMIP=$OUTBOX remote_cmd $*
}

init_2106() {
	if [ "$LLC_TRG" = "ext" ]; then
		out_cmd $AO trg=1,0,1
		echo setting ext trigger $(out_cmd $AO trg)
	else
		out_cmd $AO trg=1,1,1
	fi
	if [ "$LLC_CLK" = "ext" ]; then
		echo setting external clock / $EXTCLKDIV
		out_cmd $AO clk=1,0,1
		out_cmd $AO clkdiv=$EXTCLKDIV
	else
		out_cmd $AO clk=0,0,0
		let aodiv="$INTCLKDIV*2/3"
		echo "setting clkdiv $aodiv"	
		out_cmd $AO clkdiv=$aodiv
		echo setting internal clock / $INTCLKDIV
	fi
	out_cmd $SC distributor sites=$AO,$DO comms=1 pad=15 off
	out_cmd $SC distributor sites=$AO,$DO comms=1 pad=15 on
	out_cmd $AO lotide 256	
        out_cmd $AO twos_comp_encoding=1       	
	
        out_cmd $DO lotide 256
        out_cmd $DO clk=1,2,1
        out_cmd $DO byte_is_output=1,1,1,1
        out_cmd $DO mode=1
}

start_stream() {
	if [ "$LLC_TRG" != "ext" ]; then
		out_cmd $SC soft_trigger
	fi
}
	

cleanup_2106() {
	echo "remote_cmd $SC distributor off"
	out_cmd $SC distributor off
}


