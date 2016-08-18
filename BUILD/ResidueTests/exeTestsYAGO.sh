testDir="/home/yago/code/RegistrationPipelineFerran/BUILD"
modelsDir="/home/yago/code/models/ResidueTests"
outputFilePrefix=$testDir"/ResidueTests/test4PAPAPA"
outputFilePrefix2=$testDir"/ResidueTests/test4APAPA"

# loop over kdtree thresholds and gridtreecellfactors, for every model

# TEST WITH WITH THRS DEPENDING ON M -----------------------------
for kdThreshold in 1 2 3 4 5 6 7 8 9 10 
#for kdThreshold in 50
do
	for cellFactor in 20 50 70 100 120 150 200 250 500 750
#	for cellFactor in 150
	do
		#test Bremen
		printf "\nStarting Bremen\n"
		date
		START=$(date +%s)
		echo -n "$kdThreshold; $cellFactor; " >> $outputFilePrefix"bremen.xls" # the output of the pipeline should be changed so it only produces the numbers that we want (and maybe structure identifiers) in a single line)
		bash $testDir/ResidueTests/computeModel.sh $testDir $testDir/ResidueTests/bremen/ $modelsDir/bremen_target.ply $modelsDir/bremen_source.ply $testDir/ResidueTests/bremen/matrixBremen.xls $outputFilePrefix"bremen.xls" $kdThreshold $cellFactor
    echo -n -e "\n" >> $outputFilePrefix"bremen.xls"
    END=$(date +%s)
		DIFF=$(( $END - $START ))
		printf "Bremen tested IN $DIFF\n\n"

		#test Bust
		printf "\nStarting Bust\n"
		date
		START=$(date +%s)
		echo -n "$kdThreshold; $cellFactor; " >> $outputFilePrefix"bust.xls"
		bash $testDir/ResidueTests/computeModel.sh $testDir $testDir/ResidueTests/bust/ $modelsDir/bust0.ply $modelsDir/bust1.ply $testDir/ResidueTests/bust/matrixBust.xls $outputFilePrefix"bust.xls" $kdThreshold $cellFactor
    echo -n -e "\n" >> $outputFilePrefix"bust.xls"
    END=$(date +%s)
		DIFF=$(( $END - $START ))
		printf "Bust tested IN $DIFF\n\n"

		#test Buddha
		printf "\nStarting Buddha\n"
		date
		START=$(date +%s)
		echo -n "$kdThreshold; $cellFactor; " >> $outputFilePrefix"buddha.xls"
		bash $testDir/ResidueTests/computeModel.sh $testDir $testDir/ResidueTests/buddha/ $modelsDir/buddha0.ply $modelsDir/buddha1.ply $testDir/ResidueTests/buddha/matrixBuddha.xls $outputFilePrefix"buddha.xls" $kdThreshold $cellFactor
    echo -n -e "\n" >> $outputFilePrefix"buddha.xls"
    END=$(date +%s)
		DIFF=$(( $END - $START ))
		printf "Buddha tested IN $DIFF\n\n"


		#test Bunny
		printf "\nStarting Bunny\n"
		date
		START=$(date +%s)
		echo -n "$kdThreshold; $cellFactor; " >> $outputFilePrefix"bunny.xls"
		bash $testDir/ResidueTests/computeModel.sh $testDir $testDir/ResidueTests/bun/ $modelsDir/bun0.ply $modelsDir/bun1.ply $testDir/ResidueTests/bun/matrixBun.xls $outputFilePrefix"bunny.xls" $kdThreshold $cellFactor
    echo -n -e "\n" >> $outputFilePrefix"bunny.xls"
    END=$(date +%s)
		DIFF=$(( $END - $START ))
		printf "Bunny tested IN $DIFF\n\n"


		#test Joints
		printf "\nStarting Joints\n"
		date
		START=$(date +%s)
		echo -n "$kdThreshold; $cellFactor; " >> $outputFilePrefix"joints.xls"
		bash $testDir/ResidueTests/computeModel.sh $testDir $testDir/ResidueTests/joints/ $modelsDir/joints.ply $modelsDir/joint1.ply $testDir/ResidueTests/joints/matrixJoints.xls $outputFilePrefix"joints.xls" $kdThreshold $cellFactor
    echo -n -e "\n" >> $outputFilePrefix"joints.xls"
    END=$(date +%s)
		DIFF=$(( $END - $START ))
		printf "Joints tested IN $DIFF\n\n"

	done
done

## TEST WITH WITH FIX THRS -----------------------------
#for kdThreshold in 1 5 10 25 50 100 200 300 500
##for kdThreshold in 50
#do
#	for cellFactor in 20 50 70 100 120
##	for cellFactor in 150
#	do
#		#test Bremen
#		printf "\nStarting Bremen\n"
#		date
#		START=$(date +%s)
#		echo -n "$kdThreshold; $cellFactor; " >> $outputFilePrefix2"bremen.xls" # the output of the pipeline should be changed so it only produces the numbers that we want (and maybe structure identifiers) in a single line)
#		bash $testDir/ResidueTests/computeModel2.sh $testDir $testDir/ResidueTests/bremen/ $modelsDir/bremen_target.ply $modelsDir/bremen_source.ply $testDir/ResidueTests/bremen/matrixBremen.xls $outputFilePrefix2"bremen.xls" $kdThreshold $cellFactor
#    echo -n -e "\n" >> $outputFilePrefix2"bremen.xls"
#    END=$(date +%s)
#		DIFF=$(( $END - $START ))
#		printf "Bremen tested IN $DIFF\n\n"
#
#
#		#test Bust
#		printf "\nStarting Bust\n"
#		date
#		START=$(date +%s)
#		echo -n "$kdThreshold; $cellFactor; " >> $outputFilePrefix2"bust.xls"
#		bash $testDir/ResidueTests/computeModel2.sh $testDir $testDir/ResidueTests/bust/ $modelsDir/bust0.ply $modelsDir/bust1.ply $testDir/ResidueTests/bust/matrixBust.xls $outputFilePrefix2"bust.xls" $kdThreshold $cellFactor
#    echo -n -e "\n" >> $outputFilePrefix2"bust.xls"
#    END=$(date +%s)
#		DIFF=$(( $END - $START ))
#		printf "Bust tested IN $DIFF\n\n"
#
#		#test Buddha
#		printf "\nStarting Buddha\n"
#		date
#		START=$(date +%s)
#		echo -n "$kdThreshold; $cellFactor; " >> $outputFilePrefix2"buddha.xls"
#		bash $testDir/ResidueTests/computeModel2.sh $testDir $testDir/ResidueTests/buddha/ $modelsDir/buddha0.ply $modelsDir/buddha1.ply $testDir/ResidueTests/buddha/matrixBuddha.xls $outputFilePrefix2"buddha.xls" $kdThreshold $cellFactor
#    echo -n -e "\n" >> $outputFilePrefix2"buddha.xls"
#    END=$(date +%s)
#		DIFF=$(( $END - $START ))
#		printf "Buddha tested IN $DIFF\n\n"
#
#
#		#test Bunny
#		printf "\nStarting Bunny\n"
#		date
#		START=$(date +%s)
#		echo -n "$kdThreshold; $cellFactor; " >> $outputFilePrefix2"bunny.xls"
#		bash $testDir/ResidueTests/computeModel2.sh $testDir $testDir/ResidueTests/bun/ $modelsDir/bun0.ply $modelsDir/bun1.ply $testDir/ResidueTests/bun/matrixBun.xls $outputFilePrefix2"bunny.xls" $kdThreshold $cellFactor
#    echo -n -e "\n" >> $outputFilePrefix2"bunny.xls"
#    END=$(date +%s)
#		DIFF=$(( $END - $START ))
#		printf "Bunny tested IN $DIFF\n\n"
#
#
#		#test Joints
#		printf "\nStarting Joints\n"
#		date
#		START=$(date +%s)
#		echo -n "$kdThreshold; $cellFactor; " >> $outputFilePrefix2"joints.xls"
#		bash $testDir/ResidueTests/computeModel2.sh $testDir $testDir/ResidueTests/joints/ $modelsDir/joints.ply $modelsDir/joint1.ply $testDir/ResidueTests/joints/matrixJoints.xls $outputFilePrefix2"joints.xls" $kdThreshold $cellFactor
#    echo -n -e "\n" >> $outputFilePrefix2"joints.xls"
#    END=$(date +%s)
#		DIFF=$(( $END - $START ))
#		printf "Joints tested IN $DIFF\n\n"
#
#	done
#done
