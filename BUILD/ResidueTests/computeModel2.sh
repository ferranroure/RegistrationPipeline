#!/usr/bin/env bash
# First gather file names and other parameters

execDir=$1 #directory containing the executable file
outputDir=$2 #directory where files will be outputted
# give the following 3 in absolute routes.
targetFile=$3
sourceFile=$4
matrix=$5
outputFile=$6

printf "[[ DIRS: ]] \n"
printf "Exec: $execDir \n"
printf "Output: $outputDir\n"
printf "[[ FILES ]]\n"
printf "TARGET: $targetFile \n"
printf "SOURCE: $sourceFile \n"
printf "MATRIX: $matrix \n"
printf "OUTPUT: $outputFile\n"

datastruct[0]="kdtree"
datastruct[1]="gridtree"
datastruct[2]="trihash"
datastruct[3]="gridtree"
datastruct[4]="compressedOctree"
datastruct[5]="gridtree"
datastruct[6]="S4PCSkdtree"
datastruct[7]="gridtree"

for i in {0..7}
do
  j=$((i-1))
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
        <name>"${datastruct[$i]}"</name>
        <params>
            <param name='internDS' value='"${datastruct[$j]}"' />
        </params>
    </dataStructure>

      <generalProperties>
          <percOfPoints>1</percOfPoints>
          <nnErrorFactor>2</nnErrorFactor>
          <percOfNoise>0</percOfNoise> <!-- normalized % of MMD (perc * MMD) -->
          <normalizeModels>false</normalizeModels>
      </generalProperties>
  </params>" >> params.xml

  $execDir/Pipeline params.xml 1 $matrix >> $outputFile
  rm -f params.xml
done
