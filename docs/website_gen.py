import fog_x
import numpy as np 
import cv2 
import io 
import subprocess
import os 
from datetime import datetime

DATASETS = [
    "demo_ds"
]

viz_keys = [
    "/image/base", 
    "/image/top",
]

OUTPUT_FPS = 1

# for dataset_name in DATASETS:
dataset_name = "demo_ds"
dataset_df = fog_x.dataset.Dataset(
    name=dataset_name,
    path="s3://cloud-robotics-workshop",
)
# e = dataset.new_episode()   
# e.add_image("image", np.random.rand(100, 100, 3) * 255)


def convert_to_h264(input_file, output_file):
    
    # FFmpeg command to convert video to H.264
    command = [
        'ffmpeg',
        '-i', input_file,    # Input file
        '-loglevel', 'error', # Suppress the logs
        '-vcodec', 'h264', # Specify the codec
        output_file          # Output file
    ]
    subprocess.run(command)

view_name_to_view_id = {
    "all": "view-0",
    "segmented": "view-1",
    "base": "view-2",
    "top": "view-3",
    "other": "view-4"
}

dataset_name_to_dataset_id = {}
ds_counter = 1
for dataset in DATASETS:
    dataset_name_to_dataset_id[dataset] = "dataset-" + str(ds_counter)

dataset_h5_str = ""
for dataset in DATASETS:
    dataset_h5_str += f'''
          <div class="checkbox-container">
            <label for="dataset-{dataset_name_to_dataset_id[dataset]}">
              <input type="checkbox" class="dataset-checkbox" id="dataset-{dataset_name_to_dataset_id[dataset]}">
              {dataset} (<span id="dataset-{dataset_name_to_dataset_id[dataset]}-count">0</span>)
            </label>
          </div>
    '''

 # JSON.parse('{"dataset_id": "dataset-1", "task_id": "task-2", "object_id": "object-10", "view_id": "view-3"}'),
json_h5_str = ""
episode_id = 0
dataset_name = "demo_ds"
for episode in dataset_df.read_by(
    episode_info=dataset_df.get_episode_info()
):
    episode_ts = episode.collect()["Timestamp"]
    begin_datetime = datetime.fromtimestamp(episode_ts[0]/1000000000).strftime('%c')
    end_datetime = datetime.fromtimestamp(episode_ts[-1]/1000000000).strftime('%c')

    for key in viz_keys:
        json_info = dict()
        writer = None 
        filename = f"{dataset_name}_{episode_id}_{key}.mp4".replace("/", "_")
        json_info["poster"] = f"videos/{filename.replace('.mp4', '.jpg')}"
        json_info["src"] = f"videos/{filename}"
        json_info["info"] = "Episode: " + str(episode_id) + "<br>" + "Begin: " + begin_datetime + "<br>" + "End: " + end_datetime
        json_info["dataset_id"] = dataset_name_to_dataset_id[dataset_name]
        json_info["view_id"] = view_name_to_view_id[key.split("/")[2]]
        json_str = str(json_info).replace("'", '"')
        json_h5_str += f"JSON.parse('{json_str}'),\n"
        
        temp_vid_output_path = f"/tmp/{filename}"
        for frame in episode.collect()[key]:
            image = np.load(io.BytesIO(frame))
      
            frame_size = (image.shape[1], image.shape[0])
            if writer is None:
                writer = cv2.VideoWriter(
                    temp_vid_output_path,
                    cv2.VideoWriter_fourcc(*"mp4v"),
                    OUTPUT_FPS,
                    frame_size
                )
            writer.write(image)
        writer.release()
        final_output_path = f"./videos/{filename}"
        if not os.path.exists(temp_vid_output_path):
            convert_to_h264(temp_vid_output_path, )
        
    episode_id += 1





html_str = "\n".join([
'''
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Open-X Dataset Visualizer</title>
  <link rel="stylesheet" href="https://stackpath.bootstrapcdn.com/bootstrap/4.3.1/css/bootstrap.min.css">
  <style>
    #page-overlay {
      position: fixed;
      top: 0;
      left: 0;
      width: 100%;
      height: 100%;
      background-color: rgba(0, 0, 0, 0.5); /* Semi-transparent black */
      z-index: 9999; /* Ensure it's on top */
    }

    .checkbox-container {
      font-size: small;
      margin-bottom: -0.75rem;
      width: fit-content;
    }

    @media (min-width: 768px) {
      #video-grid {
        overflow-y: scroll; /* Always show the vertical scrollbar */
        max-height: 90vh;
      }
    }

    .video-overlay {
      position: absolute;
      top: 0;
      left: 0;
      width: 100%;
      height: 100%;
      background-color: rgba(0, 0, 0, 0.65);
      opacity: 0;
      transition: opacity 0.3s ease;
      color: white; /* White text */
      display: flex;
      justify-content: center;
      text-align: center;
      align-items: center;
      font-size: 0.75rem;
    }

    .video-overlay:hover {
      opacity: 1;
    }
  </style>
</head>
<body>
  <div id="page-overlay"></div>

  <div class="container mt-3">
    <div class="row">
      <h1 class="d-md-block d-none">Fog-RTX Dataset Visualizer</h1>
      <h3 class="d-block d-md-none">Fog-RTX Dataset Visualizer</h3>
      
      <p class="d-md-block d-none ml-3 mb-0 align-self-center">(Collected at ICRA 2024 Cloud Robotics-FogROS2 Tutorial; website template from DROID Visualizer)</p>
      <p class="d-block d-md-none small">(Collected at ICRA 2024 Cloud Robotics-FogROS2 Tutorial; website template from DROID Visualizer)</p>
      
    </div>

    <div class="row">
      <div class="col-md-2 px-1 col-12">
        <div class="mb-2 pr-5">
          <button class="btn btn-outline-dark btn-sm" id="resample-button" style="display: none;">Resample</button>
          <p class="small mb-2" id="info-text"></p>
        </div>

        <div class="mb-2" id="view-checkbox-group">
          <h5 class="mb-1">View Types</h5>

          <div class="checkbox-container">
            <label for="view-all-checkbox">
              <input type="checkbox" id="view-all-checkbox" checked>
              All (<span id="view-all-count">0</span>)
            </label>
          </div>
        
          <div class="checkbox-container">
            <label for="view-1">
              <input type="checkbox" class="view-checkbox" id="view-1">
              Segmented (<span id="view-1-count">0</span>)
            </label>
          </div>
        
          <div class="checkbox-container">
            <label for="view-2">
              <input type="checkbox" class="view-checkbox" id="view-2">
              Base (<span id="view-2-count">0</span>)
            </label>
          </div>

          <div class="checkbox-container">
            <label for="view-3">
              <input type="checkbox" class="view-checkbox" id="view-3">
              Top (<span id="view-3-count">0</span>)
            </label>
          </div>
      
          <div class="checkbox-container">
            <label for="view-4">
              <input type="checkbox" class="view-checkbox" id="view-4">
              Other (<span id="view-4-count">0</span>)
            </label>
          </div>
        </div>
      
        <div class="mb-2" id="object-checkbox-group">
          <h5 class="mb-1">Object Types</h5>

          <div class="checkbox-container">
            <label for="object-all-checkbox">
              <input type="checkbox" id="object-all-checkbox" checked>
              All (<span id="object-all-count">0</span>)
            </label>
          </div>
        
          <div class="checkbox-container">
            <label for="object-1">
              <input type="checkbox" class="object-checkbox" id="object-1">
              Marker (<span id="object-1-count">0</span>)
            </label>
          </div>
        
          <div class="checkbox-container">
            <label for="object-10">
              <input type="checkbox" class="object-checkbox" id="object-10">
              Other (<span id="object-10-count">0</span>)
            </label>
          </div>
        </div>
      
        <div class="mb-2" id="task-checkbox-group">
          <h5 class="mb-1">Task Types</h5>

          <div class="checkbox-container">
            <label for="task-all-checkbox">
              <input type="checkbox" id="task-all-checkbox" checked>
              All (<span id="task-all-count">0</span>)
            </label>
          </div>
        
          <div class="checkbox-container">
            <label for="task-1">
              <input type="checkbox" class="task-checkbox" id="task-1">
              Put (<span id="task-1-count">0</span>)
            </label>
          </div>
    

          <div class="checkbox-container">
            <label for="task-15">
              <input type="checkbox" class="task-checkbox" id="task-15">
              Other (<span id="task-15-count">0</span>)
            </label>
          </div>
        </div>
      
        <div class="mb-2" id="dataset-checkbox-group" style="overflow:scroll; height:400px;">
          <h5 class="mb-1">Dataset</h5>

          <div class="checkbox-container">
            <label for="dataset-all-checkbox">
              <input type="checkbox" id="dataset-all-checkbox" checked>
              All (<span id="dataset-all-count">0</span>)
            </label>
          </div>
'''
,
dataset_h5_str
,
'''

      </div>

      <div class="col" id="video-grid">
        <div class="row">
          <div class="col-md-3 p-1 d-flex">
            <video class="border" autoplay loop muted playsinline width="100%" type="video/mp4"></video>
            <div class="video-overlay p-3"><span></span></div>
          </div>
        
          <div class="col-md-3 p-1 d-flex">
            <video class="border" autoplay loop muted playsinline width="100%" type="video/mp4"></video>
            <div class="video-overlay p-3"><span></span></div>
          </div>
        
          <div class="col-md-3 p-1 d-flex">
            <video class="border" autoplay loop muted playsinline width="100%" type="video/mp4"></video>
            <div class="video-overlay p-3"><span></span></div>
          </div>
        
          <div class="col-md-3 p-1 d-flex">
            <video class="border" autoplay loop muted playsinline width="100%" type="video/mp4"></video>
            <div class="video-overlay p-3"><span></span></div>
          </div>
        
          <div class="col-md-3 p-1 d-flex">
            <video class="border" autoplay loop muted playsinline width="100%" type="video/mp4"></video>
            <div class="video-overlay p-3"><span></span></div>
          </div>
        
          <div class="col-md-3 p-1 d-flex">
            <video class="border" autoplay loop muted playsinline width="100%" type="video/mp4"></video>
            <div class="video-overlay p-3"><span></span></div>
          </div>
        
          <div class="col-md-3 p-1 d-flex">
            <video class="border" autoplay loop muted playsinline width="100%" type="video/mp4"></video>
            <div class="video-overlay p-3"><span></span></div>
          </div>
        
          <div class="col-md-3 p-1 d-flex">
            <video class="border" autoplay loop muted playsinline width="100%" type="video/mp4"></video>
            <div class="video-overlay p-3"><span></span></div>
          </div>
        
          <div class="col-md-3 p-1 d-flex">
            <video class="border" autoplay loop muted playsinline width="100%" type="video/mp4"></video>
            <div class="video-overlay p-3"><span></span></div>
          </div>
        
          <div class="col-md-3 p-1 d-flex">
            <video class="border" autoplay loop muted playsinline width="100%" type="video/mp4"></video>
            <div class="video-overlay p-3"><span></span></div>
          </div>
        
          <div class="col-md-3 p-1 d-flex">
            <video class="border" autoplay loop muted playsinline width="100%" type="video/mp4"></video>
            <div class="video-overlay p-3"><span></span></div>
          </div>
        
          <div class="col-md-3 p-1 d-flex">
            <video class="border" autoplay loop muted playsinline width="100%" type="video/mp4"></video>
            <div class="video-overlay p-3"><span></span></div>
          </div>
        
          <div class="col-md-3 p-1 d-flex">
            <video class="border" autoplay loop muted playsinline width="100%" type="video/mp4"></video>
            <div class="video-overlay p-3"><span></span></div>
          </div>
        
          <div class="col-md-3 p-1 d-flex">
            <video class="border" autoplay loop muted playsinline width="100%" type="video/mp4"></video>
            <div class="video-overlay p-3"><span></span></div>
          </div>
        
          <div class="col-md-3 p-1 d-flex">
            <video class="border" autoplay loop muted playsinline width="100%" type="video/mp4"></video>
            <div class="video-overlay p-3"><span></span></div>
          </div>
        
          <div class="col-md-3 p-1 d-flex">
            <video class="border" autoplay loop muted playsinline width="100%" type="video/mp4"></video>
            <div class="video-overlay p-3"><span></span></div>
          </div>
        
          <div class="col-md-3 p-1 d-flex">
            <video class="border" autoplay loop muted playsinline width="100%" type="video/mp4"></video>
            <div class="video-overlay p-3"><span></span></div>
          </div>
        
          <div class="col-md-3 p-1 d-flex">
            <video class="border" autoplay loop muted playsinline width="100%" type="video/mp4"></video>
            <div class="video-overlay p-3"><span></span></div>
          </div>
        
          <div class="col-md-3 p-1 d-flex">
            <video class="border" autoplay loop muted playsinline width="100%" type="video/mp4"></video>
            <div class="video-overlay p-3"><span></span></div>
          </div>
        
          <div class="col-md-3 p-1 d-flex">
            <video class="border" autoplay loop muted playsinline width="100%" type="video/mp4"></video>
            <div class="video-overlay p-3"><span></span></div>
          </div>
        
          <div class="col-md-3 p-1 d-flex">
            <video class="border" autoplay loop muted playsinline width="100%" type="video/mp4"></video>
            <div class="video-overlay p-3"><span></span></div>
          </div>
        
          <div class="col-md-3 p-1 d-flex">
            <video class="border" autoplay loop muted playsinline width="100%" type="video/mp4"></video>
            <div class="video-overlay p-3"><span></span></div>
          </div>
        
          <div class="col-md-3 p-1 d-flex">
            <video class="border" autoplay loop muted playsinline width="100%" type="video/mp4"></video>
            <div class="video-overlay p-3"><span></span></div>
          </div>
        
          <div class="col-md-3 p-1 d-flex">
            <video class="border" autoplay loop muted playsinline width="100%" type="video/mp4"></video>
            <div class="video-overlay p-3"><span></span></div>
          </div>
        
          <div class="col-md-3 p-1 d-flex">
            <video class="border" autoplay loop muted playsinline width="100%" type="video/mp4"></video>
            <div class="video-overlay p-3"><span></span></div>
          </div>
        
          <div class="col-md-3 p-1 d-flex">
            <video class="border" autoplay loop muted playsinline width="100%" type="video/mp4"></video>
            <div class="video-overlay p-3"><span></span></div>
          </div>
        
          <div class="col-md-3 p-1 d-flex">
            <video class="border" autoplay loop muted playsinline width="100%" type="video/mp4"></video>
            <div class="video-overlay p-3"><span></span></div>
          </div>
        
          <div class="col-md-3 p-1 d-flex">
            <video class="border" autoplay loop muted playsinline width="100%" type="video/mp4"></video>
            <div class="video-overlay p-3"><span></span></div>
          </div>
        
          <div class="col-md-3 p-1 d-flex">
            <video class="border" autoplay loop muted playsinline width="100%" type="video/mp4"></video>
            <div class="video-overlay p-3"><span></span></div>
          </div>
        
          <div class="col-md-3 p-1 d-flex">
            <video class="border" autoplay loop muted playsinline width="100%" type="video/mp4"></video>
            <div class="video-overlay p-3"><span></span></div>
          </div>
        
          <div class="col-md-3 p-1 d-flex">
            <video class="border" autoplay loop muted playsinline width="100%" type="video/mp4"></video>
            <div class="video-overlay p-3"><span></span></div>
          </div>
        
          <div class="col-md-3 p-1 d-flex">
            <video class="border" autoplay loop muted playsinline width="100%" type="video/mp4"></video>
            <div class="video-overlay p-3"><span></span></div>
          </div>
        
          <div class="col-md-3 p-1 d-flex">
            <video class="border" autoplay loop muted playsinline width="100%" type="video/mp4"></video>
            <div class="video-overlay p-3"><span></span></div>
          </div>
        
          <div class="col-md-3 p-1 d-flex">
            <video class="border" autoplay loop muted playsinline width="100%" type="video/mp4"></video>
            <div class="video-overlay p-3"><span></span></div>
          </div>
        
          <div class="col-md-3 p-1 d-flex">
            <video class="border" autoplay loop muted playsinline width="100%" type="video/mp4"></video>
            <div class="video-overlay p-3"><span></span></div>
          </div>
        
          <div class="col-md-3 p-1 d-flex">
            <video class="border" autoplay loop muted playsinline width="100%" type="video/mp4"></video>
            <div class="video-overlay p-3"><span></span></div>
          </div>
        
          <div class="col-md-3 p-1 d-flex">
            <video class="border" autoplay loop muted playsinline width="100%" type="video/mp4"></video>
            <div class="video-overlay p-3"><span></span></div>
          </div>
        
          <div class="col-md-3 p-1 d-flex">
            <video class="border" autoplay loop muted playsinline width="100%" type="video/mp4"></video>
            <div class="video-overlay p-3"><span></span></div>
          </div>
        
          <div class="col-md-3 p-1 d-flex">
            <video class="border" autoplay loop muted playsinline width="100%" type="video/mp4"></video>
            <div class="video-overlay p-3"><span></span></div>
          </div>
        
          <div class="col-md-3 p-1 d-flex">
            <video class="border" autoplay loop muted playsinline width="100%" type="video/mp4"></video>
            <div class="video-overlay p-3"><span></span></div>
          </div>
        </div>
      </div>
    </div>
  </div>

  <script>
    // Video metadata
    const videos = [
''',
json_h5_str, 
'''
    ];

    const filters = [
      'view',
      'object',
      'task',
      'dataset',
    ];

    const maxVisibleVideos = 40;

    // Cache references to DOM elements
    const infoTextElement = document.getElementById('info-text');
    const resampleButton = document.getElementById('resample-button');
    const videoGrid = document.getElementById('video-grid');
    const videoElements = document.querySelectorAll('video');
    const videoOverlays = document.querySelectorAll('.video-overlay');
    const videoOverlaySpans = document.querySelectorAll('.video-overlay span');

    // Cache references to filter DOM elements
    const filterElements = {};
    filters.forEach(filter => {
      const checkboxes = document.querySelectorAll(`.${filter}-checkbox`);
      const checkboxLabels = {};
      const checkboxCounts = {};
      checkboxes.forEach(checkbox => {
        checkboxLabels[checkbox.id] = document.querySelector(`label[for='${checkbox.id}']`);;
        checkboxCounts[checkbox.id] = document.getElementById(`${checkbox.id}-count`);
      });
      const allCheckbox = document.getElementById(`${filter}-all-checkbox`);
      const allCheckboxCount = document.getElementById(`${filter}-all-count`)
      filterElements[filter] = { checkboxes, checkboxLabels, checkboxCounts, allCheckbox, allCheckboxCount };
    });

    // Function to perform Fisher-Yates shuffle algorithm
    function fisherYatesShuffle(array) {
      for (let i = array.length - 1; i > 0; i--) {
        const j = Math.floor(Math.random() * (i + 1));
        [array[i], array[j]] = [array[j], array[i]];
      }
    }

    // Generate array of indices for shuffling
    const indices = Array.from({ length: videos.length }, (_, index) => index)
    fisherYatesShuffle(indices)

    // Create an IntersectionObserver to lazily load videos when they come into view
    const lazyVideoObserver = new IntersectionObserver(function(entries, observer) {
      entries.forEach(function(video) {
        if (video.isIntersecting) {
          video.target.load();
          observer.unobserve(video.target);
        }
      });
    });

    // Function to get selected filters
    function getSelectedValues(checkboxes) {
      return Array.from(checkboxes).filter(filter => filter.checked).map(filter => filter.id);
    }

    // Function to filter videos based on selected filters
    function filterVideos() {
      // console.time('filterVideos');

      // Stop all video observers
      lazyVideoObserver.disconnect();

      // Hide the video grid temporarily to facilitate batch-like editing of the DOM
      videoGrid.style.display = 'none';

      // Get selected values for each filter
      const selectedValuesPerFilter = {};
      filters.forEach(filter => {
        selectedValuesPerFilter[filter] = new Set(getSelectedValues(filterElements[filter].checkboxes));
      });

      // Initialize object to keep track of remaining counts for each filter value
      const remainingCounts = {};
      filters.forEach(filter => {
        remainingCounts[filter] = {};
        filterElements[filter].checkboxes.forEach(checkbox => {
          remainingCounts[filter][checkbox.id] = 0;
          remainingCounts[filter]['all'] = 0;
        });
      });

      // Iterate through shuffled videos and apply filters
      let visibleCount = 0, totalCount = 0;
      let video, matchStatusArray, selectedValues, filterIndex, filter;
      for (let i = 0; i < indices.length; i++) { // Slow
        video = videos[indices[i]];

        // Generate match status array for each filter
        matchStatusArray = [];
        for (filterIndex = 0; filterIndex < filters.length; filterIndex++) {
          filter = filters[filterIndex];
          selectedValues = selectedValuesPerFilter[filters[filterIndex]];
          matchStatusArray.push(selectedValues.size === 0 || selectedValues.has(video[`${filter}_id`]));
        }

        // Increment remaining counts applicable filter values
        for (filterIndex = 0; filterIndex < filters.length; filterIndex++) {
          // Ignoring current filter, check if all other filters match
          filter = filters[filterIndex];
          if (matchStatusArray.every((status, statusIndex) => statusIndex === filterIndex || status)) {
            remainingCounts[filter][video[`${filter}_id`]]++;
            remainingCounts[filter]['all']++;
          }
        }

        // Show videos that match all filters
        if (matchStatusArray.every(status => status)) {
          if (visibleCount < maxVisibleVideos) {
            // Set source and poster of the video element
            videoElements[visibleCount].src = video.src;
            videoElements[visibleCount].poster = video.poster;

            // Make the video element visible
            videoElements[visibleCount].style.display = '';

            // Start observing the video element for lazy loading
            lazyVideoObserver.observe(videoElements[visibleCount]);

            // Update the video overlay
            videoOverlaySpans[visibleCount].innerHTML = video.info; // Note that .textContent does not render HTML elements like <br>
            videoOverlays[visibleCount].style.display = '';

            visibleCount++;
          }
          totalCount++;
        }
      }

      // Hide any unused video containers
      for (let i = visibleCount; i < maxVisibleVideos; i++) {
        videoElements[i].style.display = 'none';
        videoElements[i].src = '';
        videoElements[i].poster = '';
        videoOverlaySpans[i].innerHTML = '';
        videoOverlays[i].style.display = 'none';
      }

      // Reveal the video grid after applying DOM edits
      videoGrid.style.display = '';

      // Update checkbox labels to show remaining counts
      filters.forEach(filter => {
        filterElements[filter].checkboxes.forEach(checkbox => {
          const label = filterElements[filter].checkboxLabels[checkbox.id];
          const count = remainingCounts[filter][checkbox.id];
          filterElements[filter].checkboxCounts[checkbox.id].textContent = count;
          if (count === 0) {
            checkbox.disabled = true;
            label.classList.add('text-muted');
          } else {
            checkbox.disabled = false;
            label.classList.remove('text-muted');
          }
        });
        filterElements[filter].allCheckboxCount.textContent = remainingCounts[filter]['all'];
      });

      // Update info text and button visibility
      if (visibleCount === totalCount) {
        infoTextElement.textContent = `Showing ${totalCount} matches`
        resampleButton.style.display = 'none';
      } else {
        infoTextElement.textContent = `Showing ${visibleCount} matches randomly sampled from ${totalCount} total matches`
        resampleButton.style.display = '';
      }

      // Reshuffle indices
      fisherYatesShuffle(indices);

      // console.timeEnd('filterVideos');
    }

    // Function to set up listener for each group of checkboxes
    function setupCheckboxGroupListener(checkboxGroupId, checkboxes, allCheckbox) {
      document.getElementById(checkboxGroupId).addEventListener('change', function(event) {
        // console.time('checkboxGroupEventListener');
        if (event.target === allCheckbox) {
          const noneSelected = Array.from(checkboxes).every(checkbox => !checkbox.checked);
          if (noneSelected) allCheckbox.checked = true;

          if (allCheckbox.checked) {
            for (const checkbox of checkboxes) {
              checkbox.checked = false;
            }
          }
          filterVideos();
        } else if (event.target.type === 'checkbox') {
          const anySelected = getSelectedValues(checkboxes).length > 0;
          allCheckbox.checked = !anySelected;
          filterVideos();
        }
        // console.timeEnd('checkboxGroupEventListener');
      });
    }

    // Set up event listener for checkbox group
    filters.forEach(filter => {
      const { checkboxes, allCheckbox } = filterElements[filter];
      setupCheckboxGroupListener(`${filter}-checkbox-group`, checkboxes, allCheckbox);
    });

    // Add a click event listener to the "Resample" button
    resampleButton.addEventListener('click', filterVideos);

    // Initial filtering of videos
    filterVideos();

    // Hide page overlay used for preventing user interaction until DOM is ready
    document.addEventListener('DOMContentLoaded', () => {
      document.getElementById('page-overlay').style.display = 'none';
    });
  </script>
</body>
</html>
'''])

with open("index.html", "w") as f:
    f.write(html_str)
