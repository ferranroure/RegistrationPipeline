#!/usr/bin/env bash
# First gather file names and other parameters

execDir=$1 #directory containing the executable file
outputDir=$2 #directory where files will be outputted
# give the following 3 in absolute routes.
targetFile=$3
sourceFile=$4
matrix=$5
outputFile=$6

# enter also other parameters
kdtreeThreshold=$7
gridCellLengthFactor=$8 #parameter onlyl to be used for the gridtree (might need to turn the single printf below in one printf for each line for this). Should be able to change the number of slotsperdimension in the gridtree data structure so the lenghts of a cell (in its smaller dimension) is a factor of epsilon.

 rm -f params.xml

printf "[[ DIRS: ]] \n"
printf "Exec: $execDir \n"
printf "Output: $outputDir\n"
printf "[[ FILES ]]\n"
printf "TARGET: $targetFile \n"
printf "SOURCE: $sourceFile \n"
printf "MATRIX: $matrix \n"
printf "OUTPUT: $outputFile\n"
printf "parameters $kdtreeThreshold $gridCellLengthFactor\n"

# definition of the data structures to be tested.

#datastruct[0]="kdtree"
#datastruct[1]="trihash"
#datastruct[2]="compressedOctree"
#datastruct[3]="S4PCSkdtree"
#datastruct[4]="gridtree"

#datastruct[0]="S4PCSkdtree"
datastruct[0]="gridtree"

for DS in "${datastruct[@]}"
do
  printf "<params>
      <files>
          <realData>true</realData>
          <infile>$targetFile</infile>
          <infile2>$sourceFile</infile2>
          <infileTemp>$outputDir/temp.ply</infileTemp>
          <outfile>$outputDir/out.ply</outfile>
          <outres>$outputDir/descriptors.txt</outres>
      </files>
      <methods>
          <detection use='false'>
              <method>ColorSpaceSampling</method>
              <properties>
                  <nSamples>1000</nSamples>
              </properties>
          </detection>

          <description use='false'>
              <method>SHOT</method>
              <properties>
                  <radiusNormalFactor>5</radiusNormalFactor>
                  <radiusSearchFactor>20</radiusSearchFactor>
                  <nNeighbours>100</nNeighbours>
              </properties>
          </description>

          <searchingStrategies use='true'>
              <method>4PCS</method>
              <properties>
                  <nCells>3</nCells>
                  <thrsFactor>1</thrsFactor>
              </properties>
          </searchingStrategies>

          <refinement use='true'>
              <method>ICP</method>
              <properties></properties>
          </refinement>

      </methods>

    <dataStructure>
        <name>$DS</name>
        <params>
            <param name='thrsKdtree' value='$kdtreeThreshold' />
            <param name='slotSizeFactor' value='$gridCellLengthFactor'/> <!-- slotPerDim = diagonal/(MMD*slotSizeFactor) -->
        </params>
    </dataStructure>

      <generalProperties>
          <percOfPoints>1</percOfPoints>
          <nnErrorFactor>3</nnErrorFactor>
          <percOfNoise>0</percOfNoise> <!-- normalized % of MMD (perc * MMD) -->
          <normalizeModels>false</normalizeModels>
      </generalProperties>
  </params>" >> params.xml


#echo "doing $execDir/Pipeline params.xml 1 $matrix"
  $execDir/Pipeline params.xml 1 $matrix >> $outputFile
  rm -f params.xml
done
