testDir="/home/ferran/MEGA/PROJECTS/RegistrationPipeline/BUILD"
modelsDir="/home/ferran/MEGA/PROJECTS/models/ResidueTests"
outputFilePrefix=$testDir"/ResidueTests/sortida"

# loop over kdtree thresholds and gridtreecellfactors, for every model

for kdThreshold in 1 5 10 25 50 100 150 200 250 300 350 400 500 750 1000
#for kdThreshold in 50
do
	for cellFactor in 10 20 50 70 100 120
	#for cellFactor in 10
	do
		#test Bremen
		printf "\nStarting Bremen\n"
		date
		START=$(date +%s)
		echo -n "$kdThreshold; $cellFactor; " >> $outputFilePrefix"bremen.txt" # the output of the pipeline should be changed so it only produces the numbers that we want (and maybe structure identifiers) in a single line)
		bash $testDir/ResidueTests/computeModel.sh $testDir $testDir/ResidueTests/bremen/ $modelsDir/bremen_target.ply $modelsDir/bremen_source.ply $testDir/ResidueTests/bremen/matrixBremen.xls $outputFilePrefix"bremen.txt" $kdThreshold $cellFactor
    echo -n -e "\n" >> $outputFilePrefix"bremen.txt"
    END=$(date +%s)
		DIFF=$(( $END - $START ))
		printf "Bremen tested IN $DIFF\n\n"


		#test Bust
		printf "\nStarting Bust\n"
		date
		START=$(date +%s)
		echo -n "$kdThreshold; $cellFactor; " >> $outputFilePrefix"bust.txt"
		bash $testDir/ResidueTests/computeModel.sh $testDir $testDir/ResidueTests/bust/ $modelsDir/bust0.ply $modelsDir/bust1.ply $testDir/ResidueTests/bust/matrixBust.xls $outputFilePrefix"bust.txt" $kdThreshold $cellFactor
    echo -n -e "\n" >> $outputFilePrefix"bust.txt"
    END=$(date +%s)
		DIFF=$(( $END - $START ))
		printf "Bust tested IN $DIFF\n\n"

		#test Buddha
		printf "\nStarting Buddha\n"
		date
		START=$(date +%s)
		echo -n "$kdThreshold; $cellFactor; " >> $outputFilePrefix"buddha.txt"
		bash $testDir/ResidueTests/computeModel.sh $testDir $testDir/ResidueTests/buddha/ $modelsDir/buddha0.ply $modelsDir/buddha1.ply $testDir/ResidueTests/buddha/matrixBuddha.xls $outputFilePrefix"buddha.txt" $kdThreshold $cellFactor
    echo -n -e "\n" >> $outputFilePrefix"buddha.txt"
    END=$(date +%s)
		DIFF=$(( $END - $START ))
		printf "Buddha tested IN $DIFF\n\n"


		#test Bunny
		printf "\nStarting Bunny\n"
		date
		START=$(date +%s)
		echo -n "$kdThreshold; $cellFactor; " >> $outputFilePrefix"bunny.txt"
		bash $testDir/ResidueTests/computeModel.sh $testDir $testDir/ResidueTests/bun/ $modelsDir/bun0.ply $modelsDir/bun1.ply $testDir/ResidueTests/bun/matrixBun.xls $outputFilePrefix"bunny.txt" $kdThreshold $cellFactor
    echo -n -e "\n" >> $outputFilePrefix"bunny.txt"
    END=$(date +%s)
		DIFF=$(( $END - $START ))
		printf "Bunny tested IN $DIFF\n\n"


		#test Joints
		printf "\nStarting Joints\n"
		date
		START=$(date +%s)
		echo -n "$kdThreshold $cellFactor " >> $outputFilePrefix"joints.txt"
		bash $testDir/ResidueTests/computeModel.sh $testDir $testDir/ResidueTests/joints/ $modelsDir/joints.ply $modelsDir/joint1.ply $testDir/ResidueTests/joints/matrixJoints.xls $outputFilePrefix"joints.txt" $kdThreshold $cellFactor
    echo -n -e "\n" >> $outputFilePrefix"joints.txt"
    END=$(date +%s)
		DIFF=$(( $END - $START ))
		printf "Joints tested IN $DIFF\n\n"

	done
done
