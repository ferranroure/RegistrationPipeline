testThrs[0]="100"
testThrs[1]="150"
testThrs[2]="200"
testThrs[3]="250"
testThrs[4]="300"
testThrs[5]="350"
testThrs[6]="400"
testThrs[7]="450"
testThrs[8]="500"


for thrs in "${testThrs[@]}"
do
  echo "<params>
      <files>
          <realData>true</realData>
          <infile>../../../../models/ResidueTests/bun0.ply</infile>
          <infile2>../../../../models/ResidueTests/bun1.ply</infile2>
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
		  <gridtreeThrs>$thrs</gridtreeThrs>
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
          <dataStructure>gridtree</dataStructure> <!-- options: kdtree, octree, compressedOctree, trihash, noDataStructure or your own -->
      </generalProperties>
  </params>" >> params.xml

  ../../Pipeline params.xml 1 matrixBun.xls
  rm -f params.xml
done
