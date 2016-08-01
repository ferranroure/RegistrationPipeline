#datastruct[0]="kdtree"
#datastruct[1]="trihash"
#datastruct[2]="compressedOctree"
#datastruct[3]="4PCSkdtree"
#datastruct[4]="gridtree"

datastruct[0]="gridtree"


for DS in "${datastruct[@]}"
do
  echo "<params>
      <files>
          <realData>true</realData>
          <infile>../../../../models/ResidueTests/medium_spiral0.ply</infile>
          <infile2>../../../../models/ResidueTests/medium_spiral0.ply</infile2>
          <infileTemp>./temp.ply</infileTemp>
          <outfile>./out.ply</outfile>
          <outres>/home/ferran/MEGA/PROJECTS/Pipeline/models/descriptors.txt</outres>
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
              <!--<method>3PS</method>-->
              <method>4PCS</method>
              <!--<method>SmartForce</method>-->
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

      <generalProperties>
          <percOfPoints>1</percOfPoints>
          <nnErrorFactor>2</nnErrorFactor>
          <percOfNoise>0</percOfNoise> <!-- normalized % of MMD (perc * MMD) -->
          <normalizeModels>false</normalizeModels>
          <dataStructure>$DS</dataStructure> <!-- options: kdtree, octree, compressedOctree, trihash, noDataStructure or your own -->
      </generalProperties>
  </params>" >> params.xml

  ../../PipelineSynth params.xml
  rm -f params.xml
done
