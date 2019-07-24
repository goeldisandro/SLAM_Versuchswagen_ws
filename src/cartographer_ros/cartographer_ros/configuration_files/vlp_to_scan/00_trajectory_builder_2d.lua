-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

TRAJECTORY_BUILDER_2D = {
-- Einstellungen zu den Inputdaten (erste Filter)
  
  use_imu_data = true,
  -- filter der LiDAR-DAten
  min_range = 1.,
  max_range = 10.,
  min_z = 0.5,
  max_z = 1.2,
 
  missing_data_ray_length = 5, -- Distanz zwischen zwei Punkten für free-Space (im rviz weiss)
  num_accumulated_range_data = 1, -- wieviele Einzelne PointCloud2 Messages zusammen fürs scan-matching verwendet werden (sollte 360° abdecken)
 
--Voxel-Filter
  voxel_filter_size = 0.05,	-- fixer Filter: Alle Punkte in einem Voxel werden zu einem zum Voxel-Mittelpunkt zusammengefasst
  adaptive_voxel_filter = {		-- adaptiver Filter
    max_length = 0.2, -- maximale grösse der Voxels
    min_num_points = 100, -- verkleinert voxels bis min-Anzahl RANGE-Punkte enthalten sind. 
    max_range = 30., -- filtert alle Punkte weiter weg
  },

  loop_closure_adaptive_voxel_filter = { -- gleich wie oben aber für das loop-closeing (eher gröber gewählt)
    max_length = 0.9,
    min_num_points = 100,
    max_range = 50.,
  },
  
-- RealTimeCorrelativeScanMacher sucht in einem gewissen Fenstern eine InitalPose um den Scan einzusetzen
-- sehr rechen intensive, aber von vorteil wenn IMU und Odom-Daten schlecht oder nicht vorhanden sind
-- überschreibt diese Sensordaten
  use_online_correlative_scan_matching = true,
  real_time_correlative_scan_matcher = {
-- Fenster grösse, je grösser desto höher der Rechenaufwand
    linear_search_window = 0.1,
    angular_search_window = math.rad(20.),
    -- je höher die cost_weight desto besser muss der Match sein für eine gewisse verschiebung
    translation_delta_cost_weight = 1e-1,
    rotation_delta_cost_weight = 1e-1,
  },

-- Findet Optimale Position auf gund der InitialPose aus Sensordaten oder aus RealTimeCorrelativeScanMacher
  ceres_scan_matcher = {
	-- Gewichtungen der Inputs (Range-Daten zu InitialPose aus Odometrie und IMU)
    occupied_space_weight = 1., -- Range-Daten
    translation_weight = 1.,
    rotation_weight = 1.,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 20,
      num_threads = 1,
    },
  },

-- Festlegen ob ein Scan eingefügt wird oder nicht
  motion_filter = {
    max_time_seconds = 1., -- nach 5 sec wird immer eingefügt
    max_distance_meters = 0.5, -- wenn zum letzten Scan ein gewisser Abstand festgestellt wird
    max_angle_radians = math.rad(1.), -- wenn zum letzten Scan eine gewisse Rotation festgestellt wird
  },

  imu_gravity_time_constant = 10., -- über diese Zeit wird die Gravitationsrichtung gemittelt

-- Einstellungen zum Erstellen der Submaps
  submaps = {
    num_range_data = 100, --grösse der Submap, so wählen, dass sie in sich konsistent ist
    grid_options_2d = {
      grid_type = "PROBABILITY_GRID",
      resolution = 0.05, -- Auflösung, je feiner desto grösser der Rechenaufwand
    },
    range_data_inserter = {
      range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D",
      probability_grid_range_data_inserter = {
        insert_free_space = true,
        hit_probability = 0.6,
        miss_probability = 0.495,
      },
    },
  },
}
