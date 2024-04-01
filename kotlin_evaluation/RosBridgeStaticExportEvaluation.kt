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

 import org.jetbrains.kotlinx.dataframe.api.toDataFrame
 import org.jetbrains.kotlinx.dataframe.api.toMap
 import org.jetbrains.letsPlot.Stat
 import org.jetbrains.letsPlot.export.ggsave
 import org.jetbrains.letsPlot.geom.geomBar
 import org.jetbrains.letsPlot.geom.geomPoint
 import org.jetbrains.letsPlot.geom.geomText
 import org.jetbrains.letsPlot.ggsize
 import org.jetbrains.letsPlot.label.ggtitle
 import org.jetbrains.letsPlot.label.labs
 import org.jetbrains.letsPlot.label.xlab
 import org.jetbrains.letsPlot.label.ylab
 import org.jetbrains.letsPlot.letsPlot
 import org.jetbrains.letsPlot.pos.positionDodge
 import org.jetbrains.letsPlot.pos.positionNudge
 import org.jetbrains.letsPlot.scale.scaleYContinuous
 import tools.aqua.stars.data.av.dataclasses.*
 import java.io.File
 import java.io.FileOutputStream
 import java.io.PrintStream
 import kotlin.io.path.Path
 import kotlin.math.abs
 import kotlin.math.max
 
 private var red = "\u001B[31m"
 private var green = "\u001B[32m"
 private var blue = "\u001B[94m"
 private var reset = "\u001B[0m"
 
 private const val verbose = false
 private const val redirectToFile = false
 private const val exportImages = false
 private const val printWaypoints = false
 
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
 //    evaluateAgainstPreExistingStaticExport(
 //        "/home/valentin/valentin-rusche-ba-code/stars-storage/STARS/STATIC/Carla/Maps/static_data__Game_Carla_Maps_${currentTown}.json",
 //        "/home/valentin/valentin-rusche-ba-code/stars-storage/STARS/STATIC/Carla/Maps/${currentTown}.json",
 //        currentTown
 //    )
 
     // Town 02
     currentTown = "Town02"
 //    evaluateAgainstPreExistingStaticExport(
 //        "/home/valentin/valentin-rusche-ba-code/stars-storage/STARS/STATIC/Carla/Maps/static_data__Game_Carla_Maps_${currentTown}.json",
 //        "/home/valentin/valentin-rusche-ba-code/stars-storage/STARS/STATIC/Carla/Maps/${currentTown}.json",
 //        currentTown
 //    )
 
     // Town 10HD
     currentTown = "Town10HD"
 
     evaluateAgainstPreExistingStaticExport(
         "/home/valentin/valentin-rusche-ba-code/stars-storage/STARS/STATIC/Carla/Maps/static_data__Game_Carla_Maps_${currentTown}.json",
         "/home/valentin/valentin-rusche-ba-code/stars-storage/STARS/STATIC/Carla/Maps/${currentTown}.json",
         currentTown
     )
 }
 
 private fun evaluateAgainstPreExistingStaticExport(
     starsExportPath: String,
     bridgeExportPath: String,
     currentTown: String
 ) {
     // Redirect standard output to a file to see all lines instead of a truncated output
     // Create a new file
     if(redirectToFile) {
         val file = File("/home/valentin/valentin-rusche-ba-code/output/$currentTown.out")
         file.parentFile.mkdirs()
         file.createNewFile()
         System.setOut(PrintStream(FileOutputStream(file)))
     }
 
     println("$currentTown: Evaluating differences between Bridge and Stars static exports")
     // ROS Bridge Blocks
     val bridgeBlocks: Sequence<Block> = loadBlocks(Path(bridgeExportPath))
     println("$currentTown: Block count in Bridge export: ${bridgeBlocks.count()}")
     // Stars Reproduction Package
     val starsBlocks: Sequence<Block> = (loadBlocks(Path(starsExportPath))).sortedBy { it.id }
     println("$currentTown: Block count in Stars export: ${starsBlocks.count()}")
 
     val onlyInBridge = bridgeBlocks.filterNot { element1 -> starsBlocks.any { it.id == element1.id } }
     val onlyInStars = starsBlocks.filterNot { element2 -> bridgeBlocks.any { it.id == element2.id } }
 
     // The 48 more blocks are roads inside a junction -> we could probably filter them out? Discuss in
     // meeting...
     println("$currentTown: Size of Block ids only in Bridge export: ${onlyInBridge.count()}")
     println("$currentTown: Size of Block ids only in Stars export: ${onlyInStars.count()}")
     val bridgeStarsBlocks = (bridgeBlocks - onlyInBridge.toSet()).sortedBy { it.id }
     println(
         "$currentTown: Filtered size of Block ids both in Bridge and Stars export: ${bridgeStarsBlocks.count()}")
     if (verbose)
         println(
             "$currentTown: Unique Elements for Bridge export: ${onlyInBridge.joinToString("; ")}\n" +
                     "$currentTown: Unique Elements for Stars export: ${onlyInStars.joinToString("; ")}")
 
     checkBlocks(bridgeStarsBlocks, starsBlocks, currentTown, onlyInBridge, onlyInStars)
 
     if(exportImages) printFilteredBlocksOfBridge(onlyInBridge, bridgeStarsBlocks, currentTown)
 
     // Keep one line space in case of multiple calls with different maps
     println()
 }
 
 private fun printFilteredBlocksOfBridge(
     onlyInBridge: Sequence<Block>,
     bothInBridgeAndStars: Sequence<Block>,
     currentTown: String
 ) {
     data class DfEntry(
         val blockId: String,
         val roadId: Int,
         val laneId: Int,
         val midpoints: List<Pair<Double, Double>>
     )
 
     val fileName = "/home/valentin/valentin-rusche-ba-code/images/${currentTown}_filtered_block_midpoints.png"
     println("$blue$currentTown: Exporting filtered Lane Midpoints to file $fileName$reset")
 
     val dfDiffClassList = mutableListOf<DfEntry>()
 
     for (block in onlyInBridge) {
         for (road in block.roads) {
             for (lane in road.lanes) {
                 val laneCoord = lane.laneMidpoints.map { it.location }.map { Pair(it.x, it.y) }
                 dfDiffClassList.add(DfEntry(block.id, road.id, lane.laneId, laneCoord))
             }
         }
     }
 
     val diffList = mutableListOf<Pair<Double, Double>>()
     for ((x, y) in dfDiffClassList.flatMap { it.midpoints }) {
         diffList.add(Pair(x, y))
     }
 
     val equalDfClassList = mutableListOf<DfEntry>()
 
     for (block in bothInBridgeAndStars) {
         for (road in block.roads) {
             for (lane in road.lanes) {
                 val laneCoord = lane.laneMidpoints.map { it.location }.map { Pair(it.x, it.y) }
                 equalDfClassList.add(DfEntry(block.id, road.id, lane.laneId, laneCoord))
             }
         }
     }
 
     val equalList = mutableListOf<Pair<Double, Double>>()
     for ((x, y) in equalDfClassList.flatMap { it.midpoints }) {
         equalList.add(Pair(x, y))
     }
 
     val diffCoordFrame = diffList.toDataFrame()
     val equalCoordFrame = equalList.toDataFrame()
 
     val p = letsPlot(diffCoordFrame.toMap()) { x = "first"; y = "second" }
     val fig = p +  ggsize(1000, 1000) + geomPoint(equalCoordFrame.toMap(), color = "red") +
             geomPoint(diffCoordFrame.toMap(), color = "blue") +
             labs(x = "x [meters]", y = "y [meters]") + ggtitle("${currentTown}: Filtered Blocks")
 
     ggsave(fig, fileName)
 }
 
 private data class LaneDifferences(
     val differentLaneIdsForRoads: MutableMap<Pair<Int, Int>, Pair<List<Int>, List<Int>>>,
     val differentLaneTypesForRoads:
     MutableMap<Pair<Int, Int>, Pair<List<LaneType>, List<LaneType>>>,
     val differentLaneWidthsForRoads: MutableMap<Pair<Int, Int>, Pair<List<Double>, List<Double>>>,
     val differentLaneLengthsForRoads: MutableMap<Pair<Int, Int>, Pair<List<Double>, List<Double>>>,
     val differentLanePredecessorsForRoads: MutableMap<Pair<Int, Int>, Pair<List<List<Lane>>, List<List<Lane>>>>,
     val differentLaneSuccessorsForRoads: MutableMap<Pair<Int, Int>, Pair<List<List<Lane>>, List<List<Lane>>>>,
     val differentLaneIntersectionsForRoads: MutableMap<Pair<Int, Int>, Pair<List<List<Lane>>, List<List<Lane>>>>,
     val differentLaneYieldsForRoads: MutableMap<Pair<Int, Int>, Pair<List<List<Lane>>, List<List<Lane>>>>,
     val differentLaneMidpointsForRoads: MutableMap<Pair<Int, Int>, Pair<List<List<LaneMidpoint>>, List<List<LaneMidpoint>>>>,
     val differentLaneSpeedLimitsForRoads: MutableMap<Pair<Int, Int>, Pair<List<List<SpeedLimit>>, List<List<SpeedLimit>>>>,
     val differentLaneLandmarkIdsForRoads: MutableMap<Pair<Int, Int>, Pair<List<Int?>, List<Int?>>>,
     val differentLaneLandmarkNamesForRoads: MutableMap<Pair<Int, Int>, Pair<List<String?>, List<String?>>>,
     val differentLaneLandmarkDistancesForRoads: MutableMap<Pair<Int, Int>, Pair<List<Double?>, List<Double?>>>,
     val differentLaneLandmarkSValuesForRoads: MutableMap<Pair<Int, Int>, Pair<List<Double?>, List<Double?>>>,
     val differentLaneLandmarkCountriesForRoads: MutableMap<Pair<Int, Int>, Pair<List<String?>, List<String?>>>,
     val differentLaneLandmarkTypesForRoads: MutableMap<Pair<Int, Int>, Pair<List<LandmarkType?>, List<LandmarkType?>>>,
     val differentLaneLandmarkValuesForRoads: MutableMap<Pair<Int, Int>, Pair<List<Double?>, List<Double?>>>,
     val differentLaneLandmarkUnitsForRoads: MutableMap<Pair<Int, Int>, Pair<List<String?>, List<String?>>>,
     val differentLaneLandmarkTextsForRoads: MutableMap<Pair<Int, Int>, Pair<List<String?>, List<String?>>>,
     val differentLaneLandmarkLocationsForRoads: MutableMap<Pair<Int, Int>, Pair<List<Location?>, List<Location?>>>,
     val differentLaneLandmarkRotationsForRoads: MutableMap<Pair<Int, Int>, Pair<List<Rotation?>, List<Rotation?>>>,
     val differentLaneContactAreasForRoads: MutableMap<Pair<Int, Int>, Pair<List<List<ContactArea>>, List<List<ContactArea>>>>,
     val differentLaneTrafficLightIdsForRoads: MutableMap<Pair<Int, Int>, Pair<List<Int?>, List<Int?>>>,
     val differentLaneTrafficLightLocationsForRoads: MutableMap<Pair<Int, Int>, Pair<List<Location?>, List<Location?>>>,
     val differentLaneTrafficLightRotationsForRoads: MutableMap<Pair<Int, Int>, Pair<List<Rotation?>, List<Rotation?>>>,
     val differentLaneTrafficLightStopLocationsForRoads: MutableMap<Pair<Int, Int>, Pair<List<List<Location>?>, List<List<Location>?>>>,
     val differentLaneLaneDirectionsForRoads: MutableMap<Pair<Int, Int>, Pair<List<LaneDirection>, List<LaneDirection>>>
 )
 
 private fun checkBlocks(
     bridgeStarsBlocks: Sequence<Block>,
     starsBlocks: Sequence<Block>,
     currentTown: String,
     onlyInBridge: Sequence<Block>,
     onlyInStars: Sequence<Block>,
 ) {
     if (bridgeStarsBlocks.count() == starsBlocks.count()) {
         // Iterate over pairs of Blocks
         val differentLanesForRoads =
             checkRoads(bridgeStarsBlocks, starsBlocks, currentTown, onlyInBridge, onlyInStars)
 
         // Check lane ids
         if (differentLanesForRoads.differentLaneIdsForRoads.isEmpty()) {
             println("$green$currentTown: Found only equal Lane Ids$reset")
         } else {
             println(
                 "$red$currentTown: Found ${differentLanesForRoads.differentLaneIdsForRoads.count()} Lane Id differences$reset"
             )
         }
 
         // check lane types
         if (differentLanesForRoads.differentLaneTypesForRoads.isEmpty()) {
             println("$green$currentTown: Found only equal Lane Types$reset")
         } else {
             println(
                 "$red$currentTown: Found ${differentLanesForRoads.differentLaneTypesForRoads.count()} Lane Type differences$reset"
             )
         }
 
         // check lane widths
         if (differentLanesForRoads.differentLaneWidthsForRoads.isEmpty()) {
             println("$green$currentTown: Found only equal Lane Widths$reset")
         } else {
             println(
                 "$red$currentTown: Found ${differentLanesForRoads.differentLaneWidthsForRoads.count()} Lane Width differences$reset"
             )
         }
 
         // check lane lengths
         if (differentLanesForRoads.differentLaneLengthsForRoads.isEmpty()) {
             println("$green$currentTown: Found only equal Lane Lengths$reset")
         } else {
             println(
                 "$red$currentTown: Found ${differentLanesForRoads.differentLaneLengthsForRoads.count()} Lane Length differences$reset"
             )
 
             if(exportImages) {
                 var delta = 0.1
                 printLaneDifferenceWithDelta(differentLanesForRoads, delta, currentTown)
                 delta = 0.5
                 printLaneDifferenceWithDelta(differentLanesForRoads, delta, currentTown)
             }
         }
 
         // check lane predecessors
         if (differentLanesForRoads.differentLanePredecessorsForRoads.isEmpty()) {
             println("$green$currentTown: Found only equal Lane Predecessors$reset")
         } else {
             println(
                 "$red$currentTown: Found ${differentLanesForRoads.differentLanePredecessorsForRoads.count()} Lane Predecessor differences$reset"
             )
         }
 
         // check lane successors
         if (differentLanesForRoads.differentLaneSuccessorsForRoads.isEmpty()) {
             println("$green$currentTown: Found only equal Lane Successors$reset")
         } else {
             println(
                 "$red$currentTown: Found ${differentLanesForRoads.differentLaneSuccessorsForRoads.count()} Lane Successor differences$reset"
             )
         }
 
         // check lane intersections
         if (differentLanesForRoads.differentLaneIntersectionsForRoads.isEmpty()) {
             println("$green$currentTown: Found only equal Lane Intersections$reset")
         } else {
             println(
                 "$red$currentTown: Found ${differentLanesForRoads.differentLaneIntersectionsForRoads.count()} Lane Intersection differences$reset"
             )
         }
 
         // check lane yields
         if (differentLanesForRoads.differentLaneYieldsForRoads.isEmpty()) {
             println("$green$currentTown: Found only equal Lane Yields$reset")
         } else {
             println(
                 "$red$currentTown: Found ${differentLanesForRoads.differentLaneYieldsForRoads.count()} Lane Yield differences$reset"
             )
         }
 
         // check lane midpoints
         if (differentLanesForRoads.differentLaneMidpointsForRoads.isEmpty()) {
             println("$green$currentTown: Found only equal Lane Midpoints$reset")
         } else {
             println(
                 "$red$currentTown: Found ${differentLanesForRoads.differentLaneMidpointsForRoads.count()} Lane Midpoint differences$reset"
             )
         }
 
         // check lane speed limits
         if (differentLanesForRoads.differentLaneSpeedLimitsForRoads.isEmpty()) {
             println("$green$currentTown: Found only equal Lane Speed Limits$reset")
         } else {
             println(
                 "$red$currentTown: Found ${differentLanesForRoads.differentLaneSpeedLimitsForRoads.count()} Lane Speed Limit differences$reset"
             )
         }
 
         // check lane landmark ids
         if (differentLanesForRoads.differentLaneLandmarkIdsForRoads.isEmpty()) {
             println("$green$currentTown: Found only equal Lane Landmark Ids$reset")
         } else {
             println(
                 "$red$currentTown: Found ${differentLanesForRoads.differentLaneLandmarkIdsForRoads.count()} Lane Landmark Id differences$reset"
             )
         }
 
         // check lane landmark names
         if (differentLanesForRoads.differentLaneLandmarkNamesForRoads.isEmpty()) {
             println("$green$currentTown: Found only equal Lane Landmark Names$reset")
         } else {
             println(
                 "$red$currentTown: Found ${differentLanesForRoads.differentLaneLandmarkNamesForRoads.count()} Lane Landmark Name differences$reset"
             )
         }
 
         // check lane landmark distances
         if (differentLanesForRoads.differentLaneLandmarkDistancesForRoads.isEmpty()) {
             println("$green$currentTown: Found only equal Lane Landmark Distances$reset")
         } else {
             println(
                 "$red$currentTown: Found ${differentLanesForRoads.differentLaneLandmarkDistancesForRoads.count()} Lane Landmark Distance differences$reset"
             )
             if(exportImages) {
                 var delta = 0.1
                 printLaneLandmarkDistanceWithDelta(differentLanesForRoads, delta, currentTown)
                 delta = 0.5
                 printLaneLandmarkDistanceWithDelta(differentLanesForRoads, delta, currentTown)
             }
         }
 
         // check lane landmark s values
         if (differentLanesForRoads.differentLaneLandmarkSValuesForRoads.isEmpty()) {
             println("$green$currentTown: Found only equal Lane Landmark S Values$reset")
         } else {
             println(
                 "$red$currentTown: Found ${differentLanesForRoads.differentLaneLandmarkSValuesForRoads.count()} Lane Landmark S Value differences$reset"
             )
             if(exportImages) {
                 var delta = 0.1
                 printLaneLandmarkSValueWithDelta(differentLanesForRoads, delta, currentTown)
                 delta = 0.5
                 printLaneLandmarkSValueWithDelta(differentLanesForRoads, delta, currentTown)
             }
         }
 
         // check lane landmark countries
         if (differentLanesForRoads.differentLaneLandmarkCountriesForRoads.isEmpty()) {
             println("$green$currentTown: Found only equal Lane Landmark Countries$reset")
         } else {
             println(
                 "$red$currentTown: Found ${differentLanesForRoads.differentLaneLandmarkCountriesForRoads.count()} Lane Landmark Country differences$reset"
             )
         }
 
         // check lane landmark types
         if (differentLanesForRoads.differentLaneLandmarkTypesForRoads.isEmpty()) {
             println("$green$currentTown: Found only equal Lane Landmark Types$reset")
         } else {
             println(
                 "$red$currentTown: Found ${differentLanesForRoads.differentLaneLandmarkTypesForRoads.count()} Lane Landmark Type differences$reset"
             )
         }
 
         // check lane landmark values
         if (differentLanesForRoads.differentLaneLandmarkValuesForRoads.isEmpty()) {
             println("$green$currentTown: Found only equal Lane Landmark Values$reset")
         } else {
             println(
                 "$red$currentTown: Found ${differentLanesForRoads.differentLaneLandmarkValuesForRoads.count()} Lane Landmark Value differences$reset"
             )
         }
 
         // check lane landmark units
         if (differentLanesForRoads.differentLaneLandmarkUnitsForRoads.isEmpty()) {
             println("$green$currentTown: Found only equal Lane Landmark Units$reset")
         } else {
             println(
                 "$red$currentTown: Found ${differentLanesForRoads.differentLaneLandmarkUnitsForRoads.count()} Lane Landmark Unit differences$reset"
             )
         }
 
         // check lane landmark texts
         if (differentLanesForRoads.differentLaneLandmarkTextsForRoads.isEmpty()) {
             println("$green$currentTown: Found only equal Lane Landmark Texts$reset")
         } else {
             println(
                 "$red$currentTown: Found ${differentLanesForRoads.differentLaneLandmarkTextsForRoads.count()} Lane Landmark Text differences$reset"
             )
         }
 
         // check lane landmark locations
         if (differentLanesForRoads.differentLaneLandmarkLocationsForRoads.isEmpty()) {
             println("$green$currentTown: Found only equal Lane Landmark Locations$reset")
         } else {
             println(
                 "$red$currentTown: Found ${differentLanesForRoads.differentLaneLandmarkLocationsForRoads.count()} Lane Landmark Location differences$reset"
             )
         }
 
         // check lane landmark rotations
         if (differentLanesForRoads.differentLaneLandmarkRotationsForRoads.isEmpty()) {
             println("$green$currentTown: Found only equal Lane Landmark Rotations$reset")
         } else {
             println(
                 "$red$currentTown: Found ${differentLanesForRoads.differentLaneLandmarkRotationsForRoads.count()} Lane Landmark Rotation differences$reset"
             )
         }
 
         // check lane contact areas
         if (differentLanesForRoads.differentLaneContactAreasForRoads.isEmpty()) {
             println("$green$currentTown: Found only equal Lane Contact Areas$reset")
         } else {
             println(
                 "$red$currentTown: Found ${differentLanesForRoads.differentLaneContactAreasForRoads.count()} Lane Contact Area differences$reset"
             )
         }
 
         // check lane traffic light ids
         if (differentLanesForRoads.differentLaneTrafficLightIdsForRoads.isEmpty()) {
             println("$green$currentTown: Found only equal Lane Traffic Light Ids$reset")
         } else {
             println(
                 "$red$currentTown: Found ${differentLanesForRoads.differentLaneTrafficLightIdsForRoads.count()} Lane Traffic Light Id differences$reset"
             )
         }
 
         // check lane traffic light locations
         if (differentLanesForRoads.differentLaneTrafficLightLocationsForRoads.isEmpty()) {
             println("$green$currentTown: Found only equal Lane Traffic Light Locations$reset")
         } else {
             println(
                 "$red$currentTown: Found ${differentLanesForRoads.differentLaneTrafficLightLocationsForRoads.count()} Lane Traffic Light Location differences$reset"
             )
         }
 
         // check lane traffic light rotations
         if (differentLanesForRoads.differentLaneTrafficLightRotationsForRoads.isEmpty()) {
             println("$green$currentTown: Found only equal Lane Traffic Light Rotations$reset")
         } else {
             println(
                 "$red$currentTown: Found ${differentLanesForRoads.differentLaneTrafficLightRotationsForRoads.count()} Lane Traffic Light Rotation differences$reset"
             )
         }
 
         // check lane traffic light stop locations
         if (differentLanesForRoads.differentLaneTrafficLightStopLocationsForRoads.isEmpty()) {
             println("$green$currentTown: Found only equal Lane Traffic Light Stop Locations$reset")
         } else {
             println(
                 "$red$currentTown: Found ${differentLanesForRoads.differentLaneTrafficLightStopLocationsForRoads.count()} Lane Traffic Light Stop Location differences$reset"
             )
         }
 
         // check lane directions
         if (differentLanesForRoads.differentLaneLaneDirectionsForRoads.isEmpty()) {
             println("$green$currentTown: Found only equal Lane Directions$reset")
         } else {
             println(
                 "$red$currentTown: Found ${differentLanesForRoads.differentLaneLaneDirectionsForRoads.count()} Lane Direction differences$reset"
             )
         }
     } else {
         println("$red$currentTown: The Block lists have different sizes.$reset")
     }
 }
 
 private fun printLaneDifferenceWithDelta(
     differentLanesForRoads: LaneDifferences,
     delta: Double,
     currentTown: String
 ) {
     val fileName = "/home/valentin/valentin-rusche-ba-code/images/${currentTown}_lane_length_difference_counts_delta_$delta.png"
     val titleString = "${currentTown}: Categorized Lane Length Differences with Delta < $delta [meters]"
     println("$blue$titleString$reset")
     var floatingPointErrorCount = 0
     var lengthDifferenceCount = 0
 
     for (entry in differentLanesForRoads.differentLaneLengthsForRoads.values) {
         for ((export, stars) in entry.first.zip(entry.second)) {
             val diff = abs(export - stars)
             if (diff < delta) {
                 floatingPointErrorCount += 1
             } else {
                 lengthDifferenceCount += 1
             }
         }
     }
 
     val data = mapOf(
         "Error Type" to listOf("Floating Point Rounding Error", "Length Difference"),
         "Count" to listOf(floatingPointErrorCount, lengthDifferenceCount)
     )
 
     val p = letsPlot(data) { x = "Error Type"; y = "Count" } + ggsize(1000, 1000)
     val fig = p + geomBar(stat = Stat.identity, position = positionDodge()) +
             geomText(
                 stat = Stat.identity, position = positionNudge(y = 1.0),
                 color = "black", size = 12
             ) { label = "Count" } +
             xlab("Differences") + ylab("Amount") +
             ggtitle(titleString) +
             scaleYContinuous(limits = Pair(0, max(floatingPointErrorCount, lengthDifferenceCount)))
 
     ggsave(fig, fileName)
 }
 
 private fun printLaneLandmarkSValueWithDelta(
     differentLanesForRoads: LaneDifferences,
     delta: Double,
     currentTown: String
 ) {
     val fileName = "/home/valentin/valentin-rusche-ba-code/images/${currentTown}_lane_landmark_s_value_delta_$delta.png"
     val titleString = "${currentTown}: Categorized Lane Landmark S Value Differences with Delta < $delta [meters]"
     println("$blue$titleString to file $fileName$reset")
     var floatingPointErrorCount = 0
     var sValueDifferenceCount = 0
 
     for (entry in differentLanesForRoads.differentLaneLandmarkSValuesForRoads.values) {
         for ((export, stars) in entry.first.zip(entry.second)) {
             val eVal = export ?: 0
             val starsVal = stars ?: 0
             val diff = abs(eVal.toInt() - starsVal.toInt())
             if(diff == 0) {
                 continue
             } else if (diff < delta) {
                 floatingPointErrorCount += 1
             } else {
                 sValueDifferenceCount += 1
             }
         }
     }
 
     val data = mapOf(
         "Error Type" to listOf("Floating Point Rounding Error", "S Value Difference"),
         "Count" to listOf(floatingPointErrorCount, sValueDifferenceCount)
     )
 
     val p = letsPlot(data) { x = "Error Type"; y = "Count" } + ggsize(1000, 1000)
     val fig = p + geomBar(stat = Stat.identity, position = positionDodge()) +
             geomText(
                 stat = Stat.identity, position = positionNudge(y = 1.0),
                 color = "black", size = 12
             ) { label = "Count" } +
             xlab("Differences") + ylab("Amount") +
             ggtitle(titleString) +
             scaleYContinuous(limits = Pair(0, max(floatingPointErrorCount, sValueDifferenceCount)))
 
     ggsave(fig, fileName)
 }
 
 private fun printLaneLandmarkDistanceWithDelta(
     differentLanesForRoads: LaneDifferences,
     delta: Double,
     currentTown: String
 ) {
     val fileName = "/home/valentin/valentin-rusche-ba-code/images/${currentTown}_lane_landmark_distance_delta_$delta.png"
     val titleString = "${currentTown}: Categorized Lane Landmark Distance Differences with Delta < $delta [meters]"
     println("$blue$titleString to file $fileName$reset")
     var floatingPointErrorCount = 0
     var distanceDifferenceCount = 0
 
     for (entry in differentLanesForRoads.differentLaneLandmarkDistancesForRoads.values) {
         for ((export, stars) in entry.first.zip(entry.second)) {
             val eVal = export ?: 0
             val starsVal = stars ?: 0
             val diff = abs(eVal.toInt() - starsVal.toInt())
             if(diff == 0) {
                 continue
             } else if (diff < delta) {
                 floatingPointErrorCount += 1
             } else {
                 distanceDifferenceCount += 1
             }
         }
     }
 
     val data = mapOf(
         "Error Type" to listOf("Floating Point Rounding Error", "Distance Difference"),
         "Count" to listOf(floatingPointErrorCount, distanceDifferenceCount)
     )
 
     val p = letsPlot(data) { x = "Error Type"; y = "Count" } + ggsize(1000, 1000)
     val fig = p + geomBar(stat = Stat.identity, position = positionDodge()) +
             geomText(
                 stat = Stat.identity, position = positionNudge(y = 1.0),
                 color = "black", size = 12
             ) { label = "Count" } +
             xlab("Differences") + ylab("Amount") +
             ggtitle(titleString) +
             scaleYContinuous(limits = Pair(0, max(floatingPointErrorCount, distanceDifferenceCount)))
 
     ggsave(fig, fileName)
 }
 
 private fun checkRoads(
     bridgeStarsBlocks: Sequence<Block>,
     starsBlocks: Sequence<Block>,
     currentTown: String,
     onlyInBridge: Sequence<Block>,
     onlyInStars: Sequence<Block>
 ): LaneDifferences {
     val differentRoadIds = mutableListOf<Pair<List<Int>, List<Int>>>()
     val differentRoadIsJunctions = mutableListOf<Pair<List<Boolean>, List<Boolean>>>()
     val differentLaneIdsForRoads = mutableMapOf<Pair<Int, Int>, Pair<List<Int>, List<Int>>>()
     val differentLaneTypesForRoads = mutableMapOf<Pair<Int, Int>, Pair<List<LaneType>, List<LaneType>>>()
     val differentLaneWidthsForRoads = mutableMapOf<Pair<Int, Int>, Pair<List<Double>, List<Double>>>()
     val differentLaneLengthsForRoads = mutableMapOf<Pair<Int, Int>, Pair<List<Double>, List<Double>>>()
     val differentLanePredecessorsForRoads = mutableMapOf<Pair<Int, Int>, Pair<List<List<Lane>>, List<List<Lane>>>>()
     val differentLaneSuccessorsForRoads = mutableMapOf<Pair<Int, Int>, Pair<List<List<Lane>>, List<List<Lane>>>>()
     val differentLaneIntersectionsForRoads = mutableMapOf<Pair<Int, Int>, Pair<List<List<Lane>>, List<List<Lane>>>>()
     val differentLaneYieldsForRoads = mutableMapOf<Pair<Int, Int>, Pair<List<List<Lane>>, List<List<Lane>>>>()
     val differentLaneMidpointsForRoads = mutableMapOf<Pair<Int, Int>, Pair<List<List<LaneMidpoint>>, List<List<LaneMidpoint>>>>()
     val differentLaneSpeedLimitsForRoads = mutableMapOf<Pair<Int, Int>, Pair<List<List<SpeedLimit>>, List<List<SpeedLimit>>>>()
     val differentLaneLandmarkIdsForRoads= mutableMapOf<Pair<Int, Int>, Pair<List<Int?>, List<Int?>>>()
     val differentLaneLandmarkNamesForRoads= mutableMapOf<Pair<Int, Int>, Pair<List<String?>, List<String?>>>()
     val differentLaneLandmarkDistancesForRoads= mutableMapOf<Pair<Int, Int>, Pair<List<Double?>, List<Double?>>>()
     val differentLaneLandmarkSValuesForRoads= mutableMapOf<Pair<Int, Int>, Pair<List<Double?>, List<Double?>>>()
     val differentLaneLandmarkCountriesForRoads= mutableMapOf<Pair<Int, Int>, Pair<List<String?>, List<String?>>>()
     val differentLaneLandmarkTypesForRoads= mutableMapOf<Pair<Int, Int>, Pair<List<LandmarkType?>, List<LandmarkType?>>>()
     val differentLaneLandmarkValuesForRoads= mutableMapOf<Pair<Int, Int>, Pair<List<Double?>, List<Double?>>>()
     val differentLaneLandmarkUnitsForRoads= mutableMapOf<Pair<Int, Int>, Pair<List<String?>, List<String?>>>()
     val differentLaneLandmarkTextsForRoads= mutableMapOf<Pair<Int, Int>, Pair<List<String?>, List<String?>>>()
     val differentLaneLandmarkLocationsForRoads= mutableMapOf<Pair<Int, Int>, Pair<List<Location?>, List<Location?>>>()
     val differentLaneLandmarkRotationsForRoads= mutableMapOf<Pair<Int, Int>, Pair<List<Rotation?>, List<Rotation?>>>()
     val differentLaneContactAreasForRoads = mutableMapOf<Pair<Int, Int>, Pair<List<List<ContactArea>>, List<List<ContactArea>>>>()
     val differentLaneTrafficLightIdsForRoads = mutableMapOf<Pair<Int, Int>, Pair<List<Int?>, List<Int?>>>()
     val differentLaneTrafficLightLocationsForRoads = mutableMapOf<Pair<Int, Int>, Pair<List<Location?>, List<Location?>>>()
     val differentLaneTrafficLightRotationsForRoads = mutableMapOf<Pair<Int, Int>, Pair<List<Rotation?>, List<Rotation?>>>()
     val differentLaneTrafficLightStopLocationsForRoads = mutableMapOf<Pair<Int, Int>, Pair<List<List<Location>?>, List<List<Location>?>>>()
     val differentLaneLaneDirectionsForRoads = mutableMapOf<Pair<Int, Int>, Pair<List<LaneDirection>, List<LaneDirection>>>()
 
 
     var laneDifferences =
         LaneDifferences(
             mutableMapOf(),
             mutableMapOf(),
             mutableMapOf(),
             mutableMapOf(),
             mutableMapOf(),
             mutableMapOf(),
             mutableMapOf(),
             mutableMapOf(),
             mutableMapOf(),
             mutableMapOf(),
             mutableMapOf(),
             mutableMapOf(),
             mutableMapOf(),
             mutableMapOf(),
             mutableMapOf(),
             mutableMapOf(),
             mutableMapOf(),
             mutableMapOf(),
             mutableMapOf(),
             mutableMapOf(),
             mutableMapOf(),
             mutableMapOf(),
             mutableMapOf(),
             mutableMapOf(),
             mutableMapOf(),
             mutableMapOf(),
             mutableMapOf()
         )
 
     for ((block1, block2) in bridgeStarsBlocks.zip(starsBlocks)) {
         // Compare the Road lists
 
         val exportRoadIds = block1.roads.map { it.id }
         val starsRoadIds = block2.roads.map { it.id }
 
         val exportRoadIsJunction = block1.roads.map { it.isJunction }
         val starsRoadIsJunction = block2.roads.map { it.isJunction }
 
         if (exportRoadIds != starsRoadIds) {
             if (verbose)
                 println(
                     "$red$currentTown: The Road Id lists of the Blocks ${block1.id} and ${block2.id} are not identical.$reset\n$currentTown: Bridge Road Ids are ${exportRoadIds}\nStars Roads Ids are        $starsRoadIds")
             differentRoadIds.add(Pair(exportRoadIds, starsRoadIds))
         }
         if (exportRoadIsJunction != starsRoadIsJunction) {
             if (verbose)
                 println(
                     "$red$currentTown: The Road isJunction attributes of the Blocks ${block1.id} and ${block2.id} are not identical.$reset\n$currentTown: Bridge isJunction attributes are ${exportRoadIsJunction}\nStars isJunction attributes are        ${starsRoadIsJunction}$reset")
             differentRoadIsJunctions.add(Pair(exportRoadIsJunction, starsRoadIsJunction))
         }
         laneDifferences =
             checkLane(
                 block1,
                 block2,
                 currentTown,
                 differentLaneIdsForRoads,
                 differentLaneTypesForRoads,
                 differentLaneWidthsForRoads,
                 differentLaneLengthsForRoads,
                 differentLanePredecessorsForRoads,
                 differentLaneSuccessorsForRoads,
                 differentLaneIntersectionsForRoads,
                 differentLaneYieldsForRoads,
                 differentLaneMidpointsForRoads,
                 differentLaneSpeedLimitsForRoads,
                 differentLaneLandmarkIdsForRoads,
                 differentLaneLandmarkNamesForRoads,
                 differentLaneLandmarkDistancesForRoads,
                 differentLaneLandmarkSValuesForRoads,
                 differentLaneLandmarkCountriesForRoads,
                 differentLaneLandmarkTypesForRoads,
                 differentLaneLandmarkValuesForRoads,
                 differentLaneLandmarkUnitsForRoads,
                 differentLaneLandmarkTextsForRoads,
                 differentLaneLandmarkLocationsForRoads,
                 differentLaneLandmarkRotationsForRoads,
                 differentLaneContactAreasForRoads,
                 differentLaneTrafficLightIdsForRoads,
                 differentLaneTrafficLightLocationsForRoads,
                 differentLaneTrafficLightRotationsForRoads,
                 differentLaneTrafficLightStopLocationsForRoads,
                 differentLaneLaneDirectionsForRoads
             )
     }
     println(
         "$blue$currentTown: Filtered ${onlyInBridge.count()} Blocks (Original Size: ${bridgeStarsBlocks.count() + onlyInBridge.count()}; New Size: ${bridgeStarsBlocks.count()}) for Bridge and ${onlyInStars.count()} Blocks for Stars (Original Size: ${starsBlocks.count() + onlyInStars.count()}; New Size: ${starsBlocks.count()})$reset")
     if (differentRoadIds.isNotEmpty() && verbose) {
         for (pair: Pair<List<Int>, List<Int>> in differentRoadIds) {
             println("$currentTown: Found unequal Road Ids: $pair")
         }
     } else {
         println("$green$currentTown: Found only equal Road Ids$reset")
     }
 
     if (differentRoadIsJunctions.isNotEmpty() && verbose) {
         for (pair: Pair<List<Boolean>, List<Boolean>> in differentRoadIsJunctions) {
             println("$currentTown: Found unequal Road isJunction attributes: $pair")
         }
     } else {
         println("$green$currentTown: Found only equal Road isJunction attributes$reset")
     }
 
     return laneDifferences
 }
 
 private fun checkLane(
     block1: Block,
     block2: Block,
     currentTown: String,
     differentLaneIdsForRoads: MutableMap<Pair<Int, Int>, Pair<List<Int>, List<Int>>>,
     differentLaneTypesForRoads: MutableMap<Pair<Int, Int>, Pair<List<LaneType>, List<LaneType>>>,
     differentLaneWidthsForRoads: MutableMap<Pair<Int, Int>, Pair<List<Double>, List<Double>>>,
     differentLaneLengthsForRoads: MutableMap<Pair<Int, Int>, Pair<List<Double>, List<Double>>>,
     differentLanePredecessorsForRoads: MutableMap<Pair<Int, Int>, Pair<List<List<Lane>>, List<List<Lane>>>>,
     differentLaneSuccessorsForRoads: MutableMap<Pair<Int, Int>, Pair<List<List<Lane>>, List<List<Lane>>>>,
     differentLaneIntersectionsForRoads: MutableMap<Pair<Int, Int>, Pair<List<List<Lane>>, List<List<Lane>>>>,
     differentLaneYieldsForRoads: MutableMap<Pair<Int, Int>, Pair<List<List<Lane>>, List<List<Lane>>>>,
     differentLaneMidpointsForRoads: MutableMap<Pair<Int, Int>, Pair<List<List<LaneMidpoint>>, List<List<LaneMidpoint>>>>,
     differentLaneSpeedLimitsForRoads: MutableMap<Pair<Int, Int>, Pair<List<List<SpeedLimit>>, List<List<SpeedLimit>>>>,
     differentLaneLandmarkIdsForRoads: MutableMap<Pair<Int, Int>, Pair<List<Int?>, List<Int?>>>,
     differentLaneLandmarkNamesForRoads: MutableMap<Pair<Int, Int>, Pair<List<String?>, List<String?>>>,
     differentLaneLandmarkDistancesForRoads: MutableMap<Pair<Int, Int>, Pair<List<Double?>, List<Double?>>>,
     differentLaneLandmarkSValuesForRoads: MutableMap<Pair<Int, Int>, Pair<List<Double?>, List<Double?>>>,
     differentLaneLandmarkCountriesForRoads: MutableMap<Pair<Int, Int>, Pair<List<String?>, List<String?>>>,
     differentLaneLandmarkTypesForRoads: MutableMap<Pair<Int, Int>, Pair<List<LandmarkType?>, List<LandmarkType?>>>,
     differentLaneLandmarkValuesForRoads: MutableMap<Pair<Int, Int>, Pair<List<Double?>, List<Double?>>>,
     differentLaneLandmarkUnitsForRoads: MutableMap<Pair<Int, Int>, Pair<List<String?>, List<String?>>>,
     differentLaneLandmarkTextsForRoads: MutableMap<Pair<Int, Int>, Pair<List<String?>, List<String?>>>,
     differentLaneLandmarkLocationsForRoads: MutableMap<Pair<Int, Int>, Pair<List<Location?>, List<Location?>>>,
     differentLaneLandmarkRotationsForRoads: MutableMap<Pair<Int, Int>, Pair<List<Rotation?>, List<Rotation?>>>,
     differentLaneContactAreasForRoads: MutableMap<Pair<Int, Int>, Pair<List<List<ContactArea>>, List<List<ContactArea>>>>,
     differentLaneTrafficLightIdsForRoads: MutableMap<Pair<Int, Int>, Pair<List<Int?>, List<Int?>>>,
     differentLaneTrafficLightLocationsForRoads: MutableMap<Pair<Int, Int>, Pair<List<Location?>, List<Location?>>>,
     differentLaneTrafficLightRotationsForRoads: MutableMap<Pair<Int, Int>, Pair<List<Rotation?>, List<Rotation?>>>,
     differentLaneTrafficLightStopLocationsForRoads: MutableMap<Pair<Int, Int>, Pair<List<List<Location>?>, List<List<Location>?>>>,
     differentLaneLaneDirectionsForRoads: MutableMap<Pair<Int, Int>, Pair<List<LaneDirection>, List<LaneDirection>>>,
 ): LaneDifferences {
     for ((road1, road2) in block1.roads.zip(block2.roads)) {
         val exportLaneIds = (road1.lanes.map { it.laneId }).sortedBy { it }
         val starsLaneIds = (road2.lanes.map { it.laneId }).sortedBy { it }
 
         if (exportLaneIds != starsLaneIds) {
             if (verbose)
                 println(
                     "$red$currentTown: The Lane Ids of the Roads ${road1.id} and ${road2.id} are not identical.$reset\n" +
                     "$currentTown: Bridge Lane Ids are ${exportLaneIds}\n$currentTown: Stars Lane Ids are  $starsLaneIds"
                 )
             differentLaneIdsForRoads[Pair(road1.id, road2.id)] = Pair(exportLaneIds, starsLaneIds)
         }
 
         val exportLaneTypes = (road1.lanes.map { it.laneType }).sortedBy { it }
         val starsLaneTypes = (road2.lanes.map { it.laneType }).sortedBy { it }
 
         if (exportLaneTypes != starsLaneTypes) {
             if (verbose)
                 println(
                     "$red$currentTown: The Lane Types of the Roads ${road1.id} and ${road2.id} are not identical.$reset\n" +
                     "$currentTown: Bridge Lane Types are ${exportLaneTypes}\n$currentTown: Stars Lane Types are  $starsLaneTypes"
                 )
             differentLaneTypesForRoads[Pair(road1.id, road2.id)] = Pair(exportLaneTypes, starsLaneTypes)
         }
 
         val exportLaneWidths = (road1.lanes.map { it.laneWidth }).sortedBy { it }
         val starsLaneWidths = (road2.lanes.map { it.laneWidth }).sortedBy { it }
 
         if (exportLaneWidths != starsLaneWidths) {
             if (verbose)
                 println(
                     "$red$currentTown: The Lane Widths of the Roads ${road1.id} and ${road2.id} are not identical.$reset\n" +
                     "$currentTown: Bridge Lane Widths are ${exportLaneWidths}\n$currentTown: Stars Lane Widths are  $starsLaneWidths"
                 )
             differentLaneWidthsForRoads[Pair(road1.id, road2.id)] =
                 Pair(exportLaneWidths, starsLaneWidths)
         }
 
         val exportLaneLengths = (road1.lanes.map { it.laneLength }).sortedBy { it }
         val starsLaneLengths = (road2.lanes.map { it.laneLength }).sortedBy { it }
 
         if (exportLaneLengths != starsLaneLengths) {
             if (verbose)
                 println(
                     "$red$currentTown: The Lane Lengths of the Roads ${road1.id} and ${road2.id} are not identical.$reset\n" +
                     "$currentTown: Bridge Lane Lengths are ${exportLaneLengths}\n$currentTown: Stars Lane Lengths are  $starsLaneLengths"
                 )
             differentLaneLengthsForRoads[Pair(road1.id, road2.id)] =
                 Pair(exportLaneLengths, starsLaneLengths)
         }
 
         val exportLanePredecessors = (road1.lanes.map { it.predecessorLanes.map { contactLaneInfo ->  contactLaneInfo.lane } }).sortedBy { lanes -> lanes.first().laneId }
         val starsLanePredecessors = (road2.lanes.map { it.predecessorLanes.map { contactLaneInfo ->  contactLaneInfo.lane } }).sortedBy { lanes -> lanes.first().laneId }
 
         if (exportLanePredecessors != starsLanePredecessors) {
             if (verbose)
                 println(
                     "$red$currentTown: The Lane Predecessors of the Roads ${road1.id} and ${road2.id} are not identical.$reset\n" +
                     "$currentTown: Bridge Lane Predecessors are ${exportLanePredecessors}\n$currentTown: Stars Lane Predecessors are  $starsLanePredecessors"
                 )
             differentLanePredecessorsForRoads[Pair(road1.id, road2.id)] =
                 Pair(exportLanePredecessors, starsLanePredecessors)
         }
 
         val exportLaneSuccessors = (road1.lanes.map { it.successorLanes.map { contactLaneInfo ->  contactLaneInfo.lane } }).sortedBy { lanes -> lanes.first().laneId }
         val starsLaneSuccessors = (road2.lanes.map { it.successorLanes.map { contactLaneInfo ->  contactLaneInfo.lane } }).sortedBy { lanes -> lanes.first().laneId }
 
         if (exportLaneSuccessors != starsLaneSuccessors) {
             if (verbose)
                 println(
                     "$red$currentTown: The Lane Successors of the Roads ${road1.id} and ${road2.id} are not identical.$reset\n" +
                     "$currentTown: Bridge Lane Successors are ${exportLaneSuccessors}\n$currentTown: Stars Lane Successors are  $starsLaneSuccessors"
                 )
             differentLaneSuccessorsForRoads[Pair(road1.id, road2.id)] =
                 Pair(exportLaneSuccessors, starsLaneSuccessors)
         }
 
         val exportLaneIntersections = (road1.lanes.map { it.intersectingLanes.map { contactLaneInfo ->  contactLaneInfo.lane } }).sortedBy { lanes -> lanes.firstOrNull()?.laneId }
         val starsLaneIntersections = (road2.lanes.map { it.intersectingLanes.map { contactLaneInfo ->  contactLaneInfo.lane } }).sortedBy { lanes -> lanes.firstOrNull()?.laneId }
 
         if (exportLaneIntersections != starsLaneIntersections) {
             if (verbose)
                 println(
                     "$red$currentTown: The Lane Intersections of the Roads ${road1.id} and ${road2.id} are not identical.$reset\n" +
                     "$currentTown: Bridge Lane Intersections are ${exportLaneIntersections}\n$currentTown: Stars Lane Intersections are  $starsLaneIntersections"
                 )
             differentLaneIntersectionsForRoads[Pair(road1.id, road2.id)] =
                 Pair(exportLaneIntersections, starsLaneIntersections)
         }
 
         val exportLaneYields = (road1.lanes.map { it.yieldLanes.map { contactLaneInfo ->  contactLaneInfo.lane } }).sortedBy { lanes -> lanes.firstOrNull()?.laneId }
         val starsLaneYields = (road2.lanes.map { it.yieldLanes.map { contactLaneInfo ->  contactLaneInfo.lane } }).sortedBy { lanes -> lanes.firstOrNull()?.laneId }
 
         if (exportLaneYields != starsLaneYields) {
             if (verbose)
                 println(
                     "$red$currentTown: The Lane Yields of the Roads ${road1.id} and ${road2.id} are not identical.$reset\n" +
                     "$currentTown: Bridge Lane Yields are ${exportLaneYields}\n$currentTown: Stars Lane Yields are  $starsLaneYields"
                 )
             differentLaneYieldsForRoads[Pair(road1.id, road2.id)] =
                 Pair(exportLaneYields, starsLaneYields)
         }
 
         val exportLaneMidpoints = (road1.lanes.map { it.laneMidpoints }).sortedWith(
             compareBy( { it.firstOrNull()?.location?.x }, { it.firstOrNull()?.location?.y }, { it.firstOrNull()?.location?.z } )
         )
         val starsLaneMidpoints = (road2.lanes.map { it.laneMidpoints }).sortedWith(
             compareBy( { it.firstOrNull()?.location?.x }, { it.firstOrNull()?.location?.y }, { it.firstOrNull()?.location?.z } )
         )
 
         if (exportLaneMidpoints != starsLaneMidpoints) {
             if (verbose)
                 println(
                     "$red$currentTown: The Lane Midpoints of the Roads ${road1.id} and ${road2.id} are not identical.$reset"
                 )
             if(printWaypoints) println("$currentTown: Bridge Lane Midpoints are ${exportLaneMidpoints}\n$currentTown: Stars Lane Midpoints are  $starsLaneMidpoints")
             differentLaneMidpointsForRoads[Pair(road1.id, road2.id)] =
                 Pair(exportLaneMidpoints, starsLaneMidpoints)
         }
 
         val exportSpeedLimits = (road1.lanes.map { it.speedLimits }).sortedBy { speedLimits -> speedLimits.firstOrNull()?.toString() }
         val starsSpeedLimits = (road2.lanes.map { it.speedLimits }).sortedBy { speedLimits -> speedLimits.firstOrNull()?.toString() }
 
         if (exportSpeedLimits != starsSpeedLimits) {
             if (verbose)
                 println(
                     "$red$currentTown: The Lane Speed Limits of the Roads ${road1.id} and ${road2.id} are not identical.$reset\n" +
                     "$currentTown: Bridge Lane Speed Limits are ${exportSpeedLimits}\n$currentTown: Stars Lane Speed Limits are  $starsSpeedLimits"
                 )
             differentLaneSpeedLimitsForRoads[Pair(road1.id, road2.id)] =
                 Pair(exportSpeedLimits, starsSpeedLimits)
         }
 
         checkLandmarks(road1, road2, currentTown,
             differentLaneLandmarkIdsForRoads,
             differentLaneLandmarkNamesForRoads,
             differentLaneLandmarkDistancesForRoads,
             differentLaneLandmarkSValuesForRoads,
             differentLaneLandmarkCountriesForRoads,
             differentLaneLandmarkTypesForRoads,
             differentLaneLandmarkValuesForRoads,
             differentLaneLandmarkUnitsForRoads,
             differentLaneLandmarkTextsForRoads,
             differentLaneLandmarkLocationsForRoads,
             differentLaneLandmarkRotationsForRoads
         )
 
         val exportContactAreas = (road1.lanes.map { it.contactAreas }).sortedBy { contactAreas -> contactAreas.firstOrNull()?.id }
         val starsContactAreas = (road2.lanes.map { it.contactAreas }).sortedBy { contactAreas -> contactAreas.firstOrNull()?.id }
 
         if (exportContactAreas != starsContactAreas) {
             if (verbose)
                 println(
                     "$red$currentTown: The Lane Contact Areas of the Roads ${road1.id} and ${road2.id} are not identical.$reset\n" +
                     "$currentTown: Bridge Lane Contact Areas are ${exportContactAreas}\n$currentTown: Stars Lane Contact Areas are  $starsContactAreas"
                 )
             differentLaneContactAreasForRoads[Pair(road1.id, road2.id)] =
                 Pair(exportContactAreas, starsContactAreas)
         }
 
         checkLaneTrafficLights(road1, road2, currentTown, differentLaneTrafficLightIdsForRoads, differentLaneTrafficLightLocationsForRoads, differentLaneTrafficLightRotationsForRoads, differentLaneTrafficLightStopLocationsForRoads)
 
         val exportLaneDirections = (road1.lanes.map { it.laneDirection }).sortedBy { laneDirection -> laneDirection.ordinal }
         val starsLaneDirections = (road2.lanes.map { it.laneDirection }).sortedBy { laneDirection -> laneDirection.ordinal }
 
         if (exportLaneDirections != starsLaneDirections) {
             if (verbose)
                 println(
                     "$red$currentTown: The Lane Directions of the Roads ${road1.id} and ${road2.id} are not identical.$reset\n" +
                     "$currentTown: Bridge Lane Directions are ${exportLaneDirections}\n$currentTown: Stars Lane Directions are  $starsLaneDirections"
                 )
             differentLaneLaneDirectionsForRoads[Pair(road1.id, road2.id)] =
                 Pair(exportLaneDirections, starsLaneDirections)
         }
     }
     return LaneDifferences(
         differentLaneIdsForRoads,
         differentLaneTypesForRoads,
         differentLaneWidthsForRoads,
         differentLaneLengthsForRoads,
         differentLanePredecessorsForRoads,
         differentLaneSuccessorsForRoads,
         differentLaneIntersectionsForRoads,
         differentLaneYieldsForRoads,
         differentLaneMidpointsForRoads,
         differentLaneSpeedLimitsForRoads,
         differentLaneLandmarkIdsForRoads,
         differentLaneLandmarkNamesForRoads,
         differentLaneLandmarkDistancesForRoads,
         differentLaneLandmarkSValuesForRoads,
         differentLaneLandmarkCountriesForRoads,
         differentLaneLandmarkTypesForRoads,
         differentLaneLandmarkValuesForRoads,
         differentLaneLandmarkUnitsForRoads,
         differentLaneLandmarkTextsForRoads,
         differentLaneLandmarkLocationsForRoads,
         differentLaneLandmarkRotationsForRoads,
         differentLaneContactAreasForRoads,
         differentLaneTrafficLightIdsForRoads,
         differentLaneTrafficLightLocationsForRoads,
         differentLaneTrafficLightRotationsForRoads,
         differentLaneTrafficLightStopLocationsForRoads,
         differentLaneLaneDirectionsForRoads
     )
 }
 
 private fun checkLandmarks(
     road1: Road,
     road2: Road,
     currentTown: String,
     differentLaneLandmarkIdsForRoads: MutableMap<Pair<Int, Int>, Pair<List<Int?>, List<Int?>>>,
     differentLaneLandmarkNamesForRoads: MutableMap<Pair<Int, Int>, Pair<List<String?>, List<String?>>>,
     differentLaneLandmarkDistancesForRoads: MutableMap<Pair<Int, Int>, Pair<List<Double?>, List<Double?>>>,
     differentLaneLandmarkSValuesForRoads: MutableMap<Pair<Int, Int>, Pair<List<Double?>, List<Double?>>>,
     differentLaneLandmarkCountriesForRoads: MutableMap<Pair<Int, Int>, Pair<List<String?>, List<String?>>>,
     differentLaneLandmarkTypesForRoads: MutableMap<Pair<Int, Int>, Pair<List<LandmarkType?>, List<LandmarkType?>>>,
     differentLaneLandmarkValuesForRoads: MutableMap<Pair<Int, Int>, Pair<List<Double?>, List<Double?>>>,
     differentLaneLandmarkUnitsForRoads: MutableMap<Pair<Int, Int>, Pair<List<String?>, List<String?>>>,
     differentLaneLandmarkTextsForRoads: MutableMap<Pair<Int, Int>, Pair<List<String?>, List<String?>>>,
     differentLaneLandmarkLocationsForRoads: MutableMap<Pair<Int, Int>, Pair<List<Location?>, List<Location?>>>,
     differentLaneLandmarkRotationsForRoads: MutableMap<Pair<Int, Int>, Pair<List<Rotation?>, List<Rotation?>>>
 ) {
     val exportLandmarkIds =
         (road1.lanes.map { it.landmarks }.map { landmarks -> landmarks.firstOrNull()?.id }).sortedBy { it }
     val starsLandmarkIds =
         (road2.lanes.map { it.landmarks }.map { landmarks -> landmarks.firstOrNull()?.id }).sortedBy { it }
 
     if (exportLandmarkIds != starsLandmarkIds) {
         if (verbose)
             println(
                 "$red$currentTown: The Lane Landmark Ids of the Roads ${road1.id} and ${road2.id} are not identical.$reset\n" +
                 "$currentTown: Bridge Lane Landmark Ids are ${exportLandmarkIds}\n$currentTown: Stars Lane Landmark Ids are  $starsLandmarkIds"
             )
         differentLaneLandmarkIdsForRoads[Pair(road1.id, road2.id)] =
             Pair(exportLandmarkIds, starsLandmarkIds)
     }
 
     val exportLandmarkNames =
         (road1.lanes.map { it.landmarks }.map { landmarks -> landmarks.firstOrNull()?.name }).sortedBy { it }
     val starsLandmarkNames =
         (road2.lanes.map { it.landmarks }.map { landmarks -> landmarks.firstOrNull()?.name }).sortedBy { it }
 
     if (exportLandmarkNames != starsLandmarkNames) {
         if (verbose)
             println(
                 "$red$currentTown: The Lane Landmarks Names of the Roads ${road1.id} and ${road2.id} are not identical.$reset\n" +
                 "$currentTown: Bridge Lane Landmark Names are ${exportLandmarkNames}\n$currentTown: Stars Lane Landmark Names are  $starsLandmarkNames"
             )
         differentLaneLandmarkNamesForRoads[Pair(road1.id, road2.id)] =
             Pair(exportLandmarkNames, starsLandmarkNames)
     }
 
     val exportLandmarkDistances =
         (road1.lanes.map { it.landmarks }.map { landmarks -> landmarks.firstOrNull()?.distance }).sortedBy { it }
     val starsLandmarkDistances =
         (road2.lanes.map { it.landmarks }.map { landmarks -> landmarks.firstOrNull()?.distance }).sortedBy { it }
 
     if (exportLandmarkDistances != starsLandmarkDistances) {
         if (verbose)
             println(
                 "$red$currentTown: The Lane Landmarks Distances of the Roads ${road1.id} and ${road2.id} are not identical.$reset\n" +
                 "$currentTown: Bridge Lane Landmark Distances are ${exportLandmarkDistances}\n$currentTown: Stars Lane Landmark Distances are  $starsLandmarkDistances"
             )
         differentLaneLandmarkDistancesForRoads[Pair(road1.id, road2.id)] =
             Pair(exportLandmarkDistances, starsLandmarkDistances)
     }
 
     val exportLandmarkSValues =
         (road1.lanes.map { it.landmarks }.map { landmarks -> landmarks.firstOrNull()?.s }).sortedBy { it }
     val starsLandmarkSValues =
         (road2.lanes.map { it.landmarks }.map { landmarks -> landmarks.firstOrNull()?.s }).sortedBy { it }
 
     if (exportLandmarkSValues != starsLandmarkSValues) {
         if (verbose)
             println(
                 "$red$currentTown: The Lane Landmarks S Values of the Roads ${road1.id} and ${road2.id} are not identical.$reset\n" +
                 "$currentTown: Bridge Lane Landmark S Values are ${exportLandmarkSValues}\n$currentTown: Stars Lane Landmark S Values are  $starsLandmarkSValues"
             )
         differentLaneLandmarkSValuesForRoads[Pair(road1.id, road2.id)] =
             Pair(exportLandmarkSValues, starsLandmarkSValues)
     }
 
     val exportLandmarkCountries =
         (road1.lanes.map { it.landmarks }.map { landmarks -> landmarks.firstOrNull()?.country }).sortedBy { it }
     val starsLandmarkCountries =
         (road2.lanes.map { it.landmarks }.map { landmarks -> landmarks.firstOrNull()?.country }).sortedBy { it }
 
     if (exportLandmarkCountries != starsLandmarkCountries) {
         if (verbose)
             println(
                 "$red$currentTown: The Lane Landmarks Countries of the Roads ${road1.id} and ${road2.id} are not identical.$reset\n" +
                 "$currentTown: Bridge Lane Landmark Countries are ${exportLandmarkCountries}\n$currentTown: Stars Lane Landmark Countries are  $starsLandmarkCountries"
             )
         differentLaneLandmarkCountriesForRoads[Pair(road1.id, road2.id)] =
             Pair(exportLandmarkCountries, starsLandmarkCountries)
     }
 
     val exportLandmarkTypes =
         (road1.lanes.map { it.landmarks }.map { landmarks -> landmarks.firstOrNull()?.type }).sortedBy { it }
     val starsLandmarkTypes =
         (road2.lanes.map { it.landmarks }.map { landmarks -> landmarks.firstOrNull()?.type }).sortedBy { it }
 
     if (exportLandmarkTypes != starsLandmarkTypes) {
         if (verbose)
             println(
                 "$red$currentTown: The Lane Landmarks Types of the Roads ${road1.id} and ${road2.id} are not identical.$reset\n" +
                 "$currentTown: Bridge Lane Landmark Types are ${exportLandmarkTypes}\n$currentTown: Stars Lane Landmark Types are  $starsLandmarkTypes"
             )
         differentLaneLandmarkTypesForRoads[Pair(road1.id, road2.id)] =
             Pair(exportLandmarkTypes, starsLandmarkTypes)
     }
 
     val exportLandmarkValues =
         (road1.lanes.map { it.landmarks }.map { landmarks -> landmarks.firstOrNull()?.value }).sortedBy { it }
     val starsLandmarkValues =
         (road2.lanes.map { it.landmarks }.map { landmarks -> landmarks.firstOrNull()?.value }).sortedBy { it }
 
     if (exportLandmarkValues != starsLandmarkValues) {
         if (verbose)
             println(
                 "$red$currentTown: The Lane Landmarks Values of the Roads ${road1.id} and ${road2.id} are not identical.$reset\n" +
                 "$currentTown: Bridge Lane Landmark Values are ${exportLandmarkValues}\n$currentTown: Stars Lane Landmark Values are  $starsLandmarkValues"
             )
         differentLaneLandmarkValuesForRoads[Pair(road1.id, road2.id)] =
             Pair(exportLandmarkValues, starsLandmarkValues)
     }
 
     val exportLandmarkUnits =
         (road1.lanes.map { it.landmarks }.map { landmarks -> landmarks.firstOrNull()?.unit }).sortedBy { it }
     val starsLandmarkUnits =
         (road2.lanes.map { it.landmarks }.map { landmarks -> landmarks.firstOrNull()?.unit }).sortedBy { it }
 
     if (exportLandmarkUnits != starsLandmarkUnits) {
         if (verbose)
             println(
                 "$red$currentTown: The Lane Landmarks Units of the Roads ${road1.id} and ${road2.id} are not identical.$reset\n" +
                 "$currentTown: Bridge Lane Landmark Units are ${exportLandmarkUnits}\n$currentTown: Stars Lane Landmark Units are  $starsLandmarkUnits"
             )
         differentLaneLandmarkUnitsForRoads[Pair(road1.id, road2.id)] =
             Pair(exportLandmarkUnits, starsLandmarkUnits)
     }
 
     val exportLandmarkTexts =
         (road1.lanes.map { it.landmarks }.map { landmarks -> landmarks.firstOrNull()?.text }).sortedBy { it }
     val starsLandmarkTexts =
         (road2.lanes.map { it.landmarks }.map { landmarks -> landmarks.firstOrNull()?.text }).sortedBy { it }
 
     if (exportLandmarkTexts != starsLandmarkTexts) {
         if (verbose)
             println(
                 "$red$currentTown: The Lane Landmarks Texts of the Roads ${road1.id} and ${road2.id} are not identical.$reset\n" +
                 "$currentTown: Bridge Lane Landmark Texts are ${exportLandmarkTexts}\n$currentTown: Stars Lane Landmark Texts are  $starsLandmarkTexts"
             )
         differentLaneLandmarkTextsForRoads[Pair(road1.id, road2.id)] =
             Pair(exportLandmarkTexts, starsLandmarkTexts)
     }
 
     val exportLandmarkLocations =
         (road1.lanes.map { it.landmarks }.map { landmarks -> landmarks.firstOrNull()?.location })
             .sortedWith(
                 compareBy( { it?.x }, { it?.y }, { it?.z } )
             )
     val starsLandmarkLocations =
         (road2.lanes.map { it.landmarks }.map { landmarks -> landmarks.firstOrNull()?.location })
             .sortedWith(
                 compareBy( { it?.x }, { it?.y }, { it?.z } )
             )
 
     if (exportLandmarkLocations != starsLandmarkLocations) {
         if (verbose)
             println(
                 "$red$currentTown: The Lane Landmarks Locations of the Roads ${road1.id} and ${road2.id} are not identical.$reset\n" +
                 "$currentTown: Bridge Lane Landmark Locations are ${exportLandmarkLocations}\n$currentTown: Stars Lane Landmark Locations are  $starsLandmarkLocations"
             )
         differentLaneLandmarkLocationsForRoads[Pair(road1.id, road2.id)] =
             Pair(exportLandmarkLocations, starsLandmarkLocations)
     }
 
     val exportLandmarkRotations =
         (road1.lanes.map { it.landmarks }.map { landmarks -> landmarks.firstOrNull()?.rotation })
             .sortedWith(
                 compareBy( { it?.yaw }, { it?.pitch }, { it?.yaw } )
             )
     val starsLandmarkRotations =
         (road2.lanes.map { it.landmarks }.map { landmarks -> landmarks.firstOrNull()?.rotation })
             .sortedWith(
                 compareBy( { it?.roll }, { it?.pitch }, { it?.yaw } )
             )
 
     if (exportLandmarkRotations != starsLandmarkRotations) {
         if (verbose)
             println(
                 "$red$currentTown: The Lane Landmarks Rotations of the Roads ${road1.id} and ${road2.id} are not identical.$reset\n" +
                 "$currentTown: Bridge Lane Landmark Rotations are ${exportLandmarkRotations}\n$currentTown: Stars Lane Landmark Rotations are  $starsLandmarkRotations"
             )
         differentLaneLandmarkRotationsForRoads[Pair(road1.id, road2.id)] =
             Pair(exportLandmarkRotations, starsLandmarkRotations)
     }
 }
 
 private fun checkLaneTrafficLights(
     road1: Road,
     road2: Road,
     currentTown: String,
     differentLaneTrafficLightIdsForRoads: MutableMap<Pair<Int, Int>, Pair<List<Int?>, List<Int?>>>,
     differentLaneTrafficLightLocationsForRoads: MutableMap<Pair<Int, Int>, Pair<List<Location?>, List<Location?>>>,
     differentLaneTrafficLightRotationsForRoads: MutableMap<Pair<Int, Int>, Pair<List<Rotation?>, List<Rotation?>>>,
     differentLaneTrafficLightStopLocationsForRoads: MutableMap<Pair<Int, Int>, Pair<List<List<Location>?>, List<List<Location>?>>>,
 ) {
     val exportTrafficLightIds = (road1.lanes.map { it.trafficLights }
         .map { staticTrafficLights -> staticTrafficLights.firstOrNull()?.id }).sortedBy { it }
     val starsTrafficLightIds = (road2.lanes.map { it.trafficLights }
         .map { staticTrafficLights -> staticTrafficLights.firstOrNull()?.id }).sortedBy { it }
 
     if (exportTrafficLightIds != starsTrafficLightIds) {
         if (verbose)
             println(
                 "$red$currentTown: The Lane Traffic Light Ids of the Roads ${road1.id} and ${road2.id} are not identical.$reset\n" +
                 "$currentTown: Bridge Lane Traffic Light Ids are ${exportTrafficLightIds}\n$currentTown: Stars Lane Traffic Light Ids are  $starsTrafficLightIds"
             )
         differentLaneTrafficLightIdsForRoads[Pair(road1.id, road2.id)] =
             Pair(exportTrafficLightIds, starsTrafficLightIds)
     }
 
     val exportTrafficLightLocations = (road1.lanes.map { it.trafficLights }
         .map { staticTrafficLights -> staticTrafficLights.firstOrNull()?.location })
         .sortedWith(
             compareBy( { it?.x }, { it?.y }, { it?.z } )
         )
 
     val starsTrafficLightLocations = (road2.lanes.map { it.trafficLights }
         .map { staticTrafficLights -> staticTrafficLights.firstOrNull()?.location })
         .sortedWith(
             compareBy( { it?.x }, { it?.y }, { it?.z } )
         )
 
 
     if (exportTrafficLightLocations != starsTrafficLightLocations) {
         if (verbose)
             println(
                 "$red$currentTown: The Lane Traffic Light Locations of the Roads ${road1.id} and ${road2.id} are not identical.$reset\n" +
                 "$currentTown: Bridge Lane Traffic Light Locations are ${exportTrafficLightLocations}\n$currentTown: Stars Lane Traffic Light Locations are  $starsTrafficLightLocations"
             )
         differentLaneTrafficLightLocationsForRoads[Pair(road1.id, road2.id)] =
             Pair(exportTrafficLightLocations, starsTrafficLightLocations)
     }
 
     val exportTrafficLightRotations = (road1.lanes.map { it.trafficLights }
         .map { staticTrafficLights -> staticTrafficLights.firstOrNull()?.rotation })
         .sortedWith(
             compareBy( { it?.roll }, { it?.pitch }, { it?.yaw } )
         )
 
     val starsTrafficLightRotations = (road2.lanes.map { it.trafficLights }
         .map { staticTrafficLights -> staticTrafficLights.firstOrNull()?.rotation })
         .sortedWith(
             compareBy( { it?.roll }, { it?.pitch }, { it?.yaw } )
         )
 
 
     if (exportTrafficLightRotations != starsTrafficLightRotations) {
         if (verbose)
             println(
                 "$red$currentTown: The Lane Traffic Light Rotations of the Roads ${road1.id} and ${road2.id} are not identical.$reset\n" +
                 "$currentTown: Bridge Lane Traffic Light Rotations are ${exportTrafficLightRotations}\n$currentTown: Stars Lane Traffic Light Rotations are  $starsTrafficLightRotations"
             )
         differentLaneTrafficLightRotationsForRoads[Pair(road1.id, road2.id)] =
             Pair(exportTrafficLightRotations, starsTrafficLightRotations)
     }
 
     val exportTrafficLightStopLocations = (road1.lanes.map { it.trafficLights }
         .map { staticTrafficLights -> staticTrafficLights.firstOrNull()?.stopLocations })
         .sortedWith(
             compareBy( { it?.firstOrNull()?.x }, { it?.firstOrNull()?.y }, { it?.firstOrNull()?.z } )
         )
 
     val starsTrafficLightStopLocations = (road2.lanes.map { it.trafficLights }
         .map { staticTrafficLights -> staticTrafficLights.firstOrNull()?.stopLocations })
         .sortedWith(
             compareBy( { it?.firstOrNull()?.x }, { it?.firstOrNull()?.y }, { it?.firstOrNull()?.z } )
         )
 
 
     if (exportTrafficLightStopLocations != starsTrafficLightStopLocations) {
         if (verbose)
             println(
                 "$red$currentTown: The Lane Traffic Light Stop Locations of the Roads ${road1.id} and ${road2.id} are not identical.$reset\n" +
                 "$currentTown: Bridge Lane Traffic Light Stop Locations are ${exportTrafficLightStopLocations}\n$currentTown: Stars Lane Traffic Light Stop Locations are  $starsTrafficLightStopLocations"
             )
         differentLaneTrafficLightStopLocationsForRoads[Pair(road1.id, road2.id)] =
             Pair(exportTrafficLightStopLocations, starsTrafficLightStopLocations)
     }
 }
 