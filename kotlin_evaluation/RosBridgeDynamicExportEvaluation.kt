/**
* Copyright (C) 2024 Valentin Rusche
*
* This program is free software: you can redistribute it and/or modify it under the terms of the GNU Affero General Public License as published by the Free Software Foundation, either version 3 of the License, or any later version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Affero General Public License for more details.
*
* You should have received a copy of the GNU Affero General Public License along with this program. If not, see <https://www.gnu.org/licenses/>
*/
 
 package tools.aqua.stars.importer.carla

 import tools.aqua.stars.data.av.dataclasses.Segment
 import tools.aqua.stars.data.av.dataclasses.TickData
 import java.io.File
 import java.io.FileOutputStream
 import java.io.PrintStream
 import kotlin.io.path.Path
 import kotlin.math.abs
 
 private var red = "\u001B[31m"
 private var green = "\u001B[32m"
 private var blue = "\u001B[94m"
 private var reset = "\u001B[0m"
 
 private const val verbose = false
 private const val redirectToFile = false
 
 fun main() {
     if(redirectToFile) {
         println("${blue}Redirecting output to file..")
         red = ""
         blue = ""
         green = ""
         reset = ""
     }
 
     // Town 01
     var currentTown = "Town01"
 //    evaluateAgainstPreExistingDynamicExport(
 //        "/home/valentin/valentin-rusche-ba-code/stars-storage/STARS/STATIC/Carla/Maps/static_data__Game_Carla_Maps_${currentTown}.json",
 //        "/home/valentin/valentin-rusche-ba-code/stars-storage/STARS/STATIC/Carla/Maps/${currentTown}.json",
 //        currentTown
 //    )
 
     // Town 02
     currentTown = "Town02"
     evaluateAgainstPreExistingDynamicExport(
         "/home/valentin/valentin-rusche-ba-code/stars-storage/STARS/STATIC/Carla/Maps/static_data__Game_Carla_Maps_${currentTown}.json",
         "/home/valentin/valentin-rusche-ba-code/stars-storage/STARS/STATIC/Carla/Maps/${currentTown}.json",
         "/home/valentin/valentin-rusche-ba-code/stars-storage/STARS/DYNAMIC/Carla/Maps/${currentTown}.json",
         "/home/valentin/valentin-rusche-ba-code/stars-storage/STARS/DYNAMIC/Carla/Maps/${currentTown}.json",
         currentTown
     )
 
     // Town 10HD
 //    currentTown = "Town10HD"
 
 //    evaluateAgainstPreExistingDynamicExport(
 //        "/home/valentin/valentin-rusche-ba-code/stars-storage/STARS/STATIC/Carla/Maps/static_data__Game_Carla_Maps_${currentTown}.json",
 //        "/home/valentin/valentin-rusche-ba-code/stars-storage/STARS/STATIC/Carla/Maps/${currentTown}.json",
 //        currentTown
 //    )
 }
 
 private fun evaluateAgainstPreExistingDynamicExport(
     starsStaticExportPath: String,
     bridgeStaticExportPath: String,
     starsDynamicExportPath: String,
     bridgeDynamicExportPath: String,
     currentTown: String
 ) {
     // Redirect standard output to a file to see all lines instead of a truncated output
     // Create a new file
     if(redirectToFile) {
         val file = File("/home/valentin/valentin-rusche-ba-code/output/dynamic/$currentTown.out")
         file.parentFile.parentFile.mkdirs()
         file.parentFile.mkdirs()
         file.createNewFile()
         System.setOut(PrintStream(FileOutputStream(file)))
     }
 
     println("$currentTown: Evaluating differences between Bridge and Stars dynamic exports")
     // ROS Bridge Dynamic Data
     val bridgeData: Sequence<Segment> = loadSegments(Path(bridgeStaticExportPath), Path(bridgeDynamicExportPath)).sortedBy { it.simulationRunId }
     println("$currentTown: Segment count in Bridge export: ${bridgeData.count()}")
     // Stars Reproduction Package
     val starsData: Sequence<Segment> = loadSegments(Path(starsStaticExportPath), Path(starsDynamicExportPath)).sortedBy { it.simulationRunId }
     println("$currentTown: Segment count in Stars export: ${starsData.count()}")
     // Keep one line space in case of multiple calls with different maps
 
     val sizeDifference = abs(bridgeData.count() - starsData.count())
     println("$currentTown: Segment count size difference: $sizeDifference")
 
     val differentSegmentVehicleIds = mutableListOf<Pair<List<Int>, List<Int>>>()
     val differentSegmentPedestrianIds = mutableListOf<Pair<List<Int>, List<Int>>>()
     val differentSegmentSimulationRunIds = mutableListOf<Pair<String, String>>()
     val differentPrimaryEntityRunIds = mutableListOf<Pair<Int, Int>>()
     val differentSegmentInitLists = mutableListOf<Pair<List<TickData>, List<TickData>>>()
 
     for ((segment1, segment2) in bridgeData.zip(starsData)) {
 
         val exportVehicleIds = segment1.vehicleIds.sorted()
         val starsVehicleIds = segment2.vehicleIds.sorted()
 
         if (exportVehicleIds != starsVehicleIds) {
             if (verbose)
                 println(
                     "$red$currentTown: The Vehicle Id lists of the Segments ${segment1.simulationRunId} and ${segment2.simulationRunId} are not identical.$reset\n" +
                     "$currentTown: Bridge Vehicle Ids are ${exportVehicleIds}\n" +
                     "$currentTown: Stars Vehicle Ids are  $starsVehicleIds"
                 )
             differentSegmentVehicleIds.add(Pair(exportVehicleIds, starsVehicleIds))
         }
 
         val exportPedestrianIds = segment1.pedestrianIds.sorted()
         val starsPedestrianIds = segment2.pedestrianIds.sorted()
 
         if (exportPedestrianIds != starsPedestrianIds) {
             if (verbose)
                 println(
                     "$red$currentTown: The Pedestrian Id lists of the Segments ${segment1.simulationRunId} and ${segment2.simulationRunId} are not identical.$reset\n" +
                     "$currentTown: Bridge Pedestrian Ids are ${exportPedestrianIds}\n" +
                     "$currentTown: Stars Pedestrian Ids are  $starsPedestrianIds"
                 )
             differentSegmentPedestrianIds.add(Pair(exportPedestrianIds, starsPedestrianIds))
         }
 
         val exportSimulationRunId = segment1.simulationRunId
         val starsSimulationRunId = segment2.simulationRunId
 
         if (exportSimulationRunId != starsSimulationRunId) {
             if (verbose)
                 println(
                     "$red$currentTown: The Simulation Run Id of the Segments ${segment1.simulationRunId} and ${segment2.simulationRunId} are not identical.$reset\n" +
                     "$currentTown: Bridge Simulation Run Id is ${exportSimulationRunId}\n" +
                     "$currentTown: Stars Simulation Run Id is  $starsSimulationRunId"
                 )
             differentSegmentSimulationRunIds.add(Pair(exportSimulationRunId, starsSimulationRunId))
         }
 
         val exportPrimaryEntityId = segment1.primaryEntityId
         val starsPrimaryEntityId = segment2.primaryEntityId
 
         if (exportPrimaryEntityId != starsPrimaryEntityId) {
             if (verbose)
                 println(
                     "$red$currentTown: The Primary Entity Id of the Segments ${segment1.simulationRunId} and ${segment2.simulationRunId} are not identical.$reset\n" +
                     "$currentTown: Bridge Primary Entity Id is ${exportPrimaryEntityId}\n" +
                     "$currentTown: Stars Primary Entity Id is  $starsPrimaryEntityId"
                 )
             differentPrimaryEntityRunIds.add(Pair(exportPrimaryEntityId, starsPrimaryEntityId))
         }
 
         val exportMainInitList = segment1.mainInitList.sortedBy { it.currentTick }
         val starsMainInitList  = segment2.mainInitList.sortedBy { it.currentTick }
 
         checkMainInitList(
             exportMainInitList,
             starsMainInitList,
             currentTown,
             segment1,
             segment2,
             differentSegmentInitLists
         )
     }
 
     println()
 }
 
 private fun checkMainInitList(
     exportMainInitList: List<TickData>,
     starsMainInitList: List<TickData>,
     currentTown: String,
     segment1: Segment,
     segment2: Segment,
     differentSegmentInitLists: MutableList<Pair<List<TickData>, List<TickData>>>
 ) {
     if (exportMainInitList != starsMainInitList) {
         if (verbose)
             println(
                 "$red$currentTown: The Main Init List of the Segments ${segment1.simulationRunId} and ${segment2.simulationRunId} are not identical.$reset\n" +
                 "$currentTown: Bridge Main Init List is ${exportMainInitList}\n" +
                 "$currentTown: Stars Main Init List is  $starsMainInitList"
             )
         differentSegmentInitLists.add(Pair(exportMainInitList, starsMainInitList))
     }
 }
 
 