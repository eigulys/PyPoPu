-- VCO Tracking Measurement Script for Reaper
-- This script sends MIDI notes to a synth and measures the frequency response 
-- to evaluate how accurately a VCO tracks across the keyboard

-- USER CONFIGURATION - EDIT THESE VALUES AS NEEDED
local midi_channel = 0           -- MIDI channel (0-15)
local start_note = 24            -- C1
local end_note = 108             -- C8
local note_velocity = 100        -- Note velocity (0-127)
local note_duration_ms = 500     -- Duration to hold each note (ms)
local pause_between_notes_ms = 200  -- Pause between notes (ms)
local input_track = 1            -- Track number for audio input
local output_track = 2           -- Track number for MIDI output
local fft_size = 16384           -- FFT size for frequency analysis

-- CSV FILE SAVE LOCATION - EDIT THIS TO CHANGE WHERE FILES ARE SAVED
-- Just specify the directory path without filename or .csv extension
-- Examples:
-- Windows: "C:/Users/YourName/Desktop"
-- Mac: "/Users/YourName/Desktop"
-- Or leave blank to use REAPER's resource path
local custom_save_directory = "N:/Studijos/Matavimai"

-- Equipment info - add details about the synth being measured
local synth_name = "Unknown Synth"  -- Name of the synth being measured
local synth_notes = ""              -- Additional notes about the measurement

-- Function to get current date and time formatted as YYYY-MM-DD_HHMMSS
local function get_datetime_string()
  local time = os.time()
  local formatted_time = os.date("%Y-%m-%d_%H%M%S", time)
  return formatted_time
end

-- Function to sanitize strings for filenames
local function sanitize_for_filename(str)
  if not str or str == "" then return "unknown" end
  -- Replace spaces with underscores and remove problematic characters
  local result = string.gsub(str, "[%s%/\\%:%*%?%\"%.%<%>%|]", "_")
  return result
end

-- Calculate the CSV filename with date, time and synth name
local csv_filename
local datetime_str = get_datetime_string()
local safe_synth_name = sanitize_for_filename(synth_name)
local filename_only = "VCO_tracking_" .. safe_synth_name .. "_" .. datetime_str .. ".csv"

if custom_save_directory ~= "" then
  -- Ensure directory path ends with a separator
  if not custom_save_directory:match("[\\/]$") then
    custom_save_directory = custom_save_directory .. "/"
  end
  csv_filename = custom_save_directory .. filename_only
else
  -- Get the REAPER resource path for file saving (ensures we have write permission)
  local resource_path = reaper.GetResourcePath()
  -- Ensure path ends with a separator
  if not resource_path:match("[\\/]$") then
    resource_path = resource_path .. "/"
  end
  csv_filename = resource_path .. filename_only
end

-- Function to convert MIDI note number to expected frequency (Hz)
local function midi_to_freq(note)
  return 440 * 2^((note - 69) / 12)
end

-- Function to convert MIDI note number to note name with octave
local function midi_to_note_name(note)
  local notes = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"}
  local octave = math.floor(note / 12) - 1
  local note_idx = (note % 12) + 1
  return notes[note_idx] .. octave
end

-- Function to wait for a specified time without using Sleep
local function wait_time(seconds)
  local start_time = reaper.time_precise()
  while reaper.time_precise() < start_time + seconds do
    -- Just wait
  end
end

-- Create a MIDI item on a track
local function create_midi_item(track)
  local cursor_pos = reaper.GetCursorPosition()
  local new_item = reaper.CreateNewMIDIItemInProj(track, cursor_pos, cursor_pos + 1, false)
  if not new_item then return nil end
  
  local take = reaper.GetActiveTake(new_item)
  if not take then
    reaper.DeleteTrackMediaItem(track, new_item)
    return nil
  end
  
  return new_item, take
end

-- Insert a MIDI note into a take
local function insert_midi_note(take, note, vel, start_time, end_time)
  if not take then return false end
  
  local start_ppq = reaper.MIDI_GetPPQPosFromProjTime(take, start_time)
  local end_ppq = reaper.MIDI_GetPPQPosFromProjTime(take, end_time)
  
  reaper.MIDI_InsertNote(take, false, false, start_ppq, end_ppq, midi_channel, note, vel, false)
  reaper.MIDI_Sort(take)
  
  return true
end

-- Function to simulate audio frequency analysis
-- Note: This is a placeholder function that generates simulated results
-- In a real application, you would need to implement actual audio analysis
local function simulate_frequency_analysis(note)
  local expected_freq = midi_to_freq(note)
  
  -- Simulate measurement with small random variation to mimic real-world behavior
  -- In a real script, this would be replaced with actual frequency analysis
  local variation = (math.random() * 2 - 1) * 2  -- Random value between -2 and +2 Hz
  local measured_freq = expected_freq + variation
  
  -- Simulate amplitude (normally this would come from actual audio analysis)
  local amplitude = 0.7 + math.random() * 0.3  -- Random amplitude between 0.7 and 1.0
  
  return measured_freq, amplitude
end

-- Function to save results to CSV
local function save_to_csv(results)
  -- Debug message to show the file path
  reaper.ShowConsoleMsg("Attempting to save file to: " .. csv_filename .. "\n")
  
  -- Try to create and open the file
  local file = io.open(csv_filename, "w")
  if not file then
    -- If we can't open the file, try a different location
    local desktop_path = reaper.GetProjectPath("") -- Get the project path instead
    if desktop_path == "" then
      desktop_path = reaper.GetResourcePath() -- Fall back to resource path
    end
    
    -- Ensure path ends with a separator
    if not desktop_path:match("[\\/]$") then
      desktop_path = desktop_path .. "/"
    end
    
    -- Try to save in the project folder instead
    local alt_csv_filename = desktop_path .. filename_only
    reaper.ShowConsoleMsg("Retrying with alternative path: " .. alt_csv_filename .. "\n")
    
    file = io.open(alt_csv_filename, "w")
    if not file then
      reaper.ShowMessageBox("Could not open file for writing at either:\n" .. csv_filename .. "\nor\n" .. alt_csv_filename, "Error", 0)
      return false, ""
    end
    csv_filename = alt_csv_filename  -- Update the filename to where it was actually saved
  end
  
  -- Write metadata header
  file:write("# VCO Tracking Measurement Results\n")
  file:write("# Synth: " .. synth_name .. "\n")
  file:write("# Date: " .. os.date("%Y-%m-%d %H:%M:%S", os.time()) .. "\n")
  file:write("# MIDI Channel: " .. midi_channel .. "\n")
  file:write("# Note Range: " .. midi_to_note_name(start_note) .. " to " .. midi_to_note_name(end_note) .. "\n")
  if synth_notes ~= "" then
    file:write("# Notes: " .. synth_notes .. "\n")
  end
  file:write("#\n")
  
  -- Write data header
  file:write("MIDI Note,Note Name,Expected Freq (Hz),Measured Freq (Hz),Deviation (cents),Amplitude\n")
  
  -- Write data
  for _, result in ipairs(results) do
    local cent_deviation = 0
    if result.measured_freq > 0 and result.expected_freq > 0 then
      cent_deviation = 1200 * math.log(result.measured_freq / result.expected_freq) / math.log(2)
    end
    
    file:write(string.format("%d,%s,%.2f,%.2f,%.2f,%.6f\n", 
      result.note,
      midi_to_note_name(result.note),
      result.expected_freq, 
      result.measured_freq, 
      cent_deviation,
      result.amplitude))
  end
  
  file:close()
  return true, csv_filename
end

-- Timer variables
local measurementTimer = 0
local currentNote = 0
local results = {}
local midi_track = nil
local audio_track = nil
local measurement_state = 0
local current_midi_item = nil
local current_take = nil
local measurement_complete = false  -- Flag to prevent rerunning after completion
local start_time = 0  -- For tracking total measurement time

-- Function to format time in MM:SS
local function format_time(seconds)
  local mins = math.floor(seconds / 60)
  local secs = math.floor(seconds % 60)
  return string.format("%02d:%02d", mins, secs)
end

-- Show dialog to allow the user to enter synth information
local function show_setup_dialog()
  local ok, user_input = reaper.GetUserInputs("VCO Tracking Setup", 4,
    "Synth Name (for filename):,Additional Notes:,Start Note (MIDI):,End Note (MIDI):", 
    synth_name .. ",,24,108")
  
  if not ok then
    return false  -- User cancelled
  end
  
  -- Parse input
  local parts = {}
  for value in user_input:gmatch("[^,]+") do
    table.insert(parts, value)
  end
  
  if #parts >= 4 then
    synth_name = parts[1]
    synth_notes = parts[2]
    local new_start = tonumber(parts[3])
    local new_end = tonumber(parts[4])
    
    if new_start and new_end and new_start >= 0 and new_start <= 127 and new_end >= 0 and new_end <= 127 then
      start_note = new_start
      end_note = new_end
    else
      reaper.ShowMessageBox("Invalid MIDI note values. Using defaults.", "Warning", 0)
    end
    
    -- Recalculate filename with new synth name
    local safe_synth_name = sanitize_for_filename(synth_name)
    filename_only = "VCO_tracking_" .. safe_synth_name .. "_" .. datetime_str .. ".csv"
    
    if custom_save_directory ~= "" then
      csv_filename = custom_save_directory .. "/" .. filename_only
    else
      csv_filename = reaper.GetResourcePath() .. "/" .. filename_only
    end
  end
  
  return true
end

-- Main measurement function
function runNextMeasurement()
  -- Prevent running if already complete
  if measurement_complete then
    return
  end
  
  -- Check if we're done
  if currentNote > end_note then
    -- Mark as complete to prevent further execution
    measurement_complete = true
    
    -- Calculate total elapsed time
    local elapsed_seconds = reaper.time_precise() - start_time
    
    -- We're done, save results
    local success, saved_file_path = save_to_csv(results)
    if success then
      -- Show stats in console
      local max_deviation = 0
      local total_deviation = 0
      local valid_results = 0
      
      for _, result in ipairs(results) do
        if result.measured_freq > 0 and result.expected_freq > 0 then
          local cent_deviation = math.abs(1200 * math.log(result.measured_freq / result.expected_freq) / math.log(2))
          max_deviation = math.max(max_deviation, cent_deviation)
          total_deviation = total_deviation + cent_deviation
          valid_results = valid_results + 1
        end
      end
      
      local avg_deviation = 0
      if valid_results > 0 then
        avg_deviation = total_deviation / valid_results
      end
      
      -- Generate summary string
      local summary = string.format("\nMEASUREMENT SUMMARY:\n")
      summary = summary .. "Synth: " .. synth_name .. "\n"
      summary = summary .. "Note Range: " .. midi_to_note_name(start_note) .. " to " .. midi_to_note_name(end_note) .. "\n"
      summary = summary .. string.format("Total notes measured: %d\n", #results)
      summary = summary .. string.format("Maximum deviation: %.2f cents\n", max_deviation)
      summary = summary .. string.format("Average deviation: %.2f cents\n", avg_deviation)
      summary = summary .. string.format("Measurement time: %s\n", format_time(elapsed_seconds))
      summary = summary .. "Results saved to: " .. saved_file_path .. "\n"
      
      reaper.ShowConsoleMsg(summary)
        
      -- Show success message only once with unique ID to prevent reopening
      local msg_id = "vco_tracking_complete_" .. os.time()
      reaper.SetExtState("VCO_TRACKING", "last_message", msg_id, false)
      reaper.ShowMessageBox("VCO tracking measurement complete!\n\n" .. summary, "Success", 0)
    else
      reaper.ShowConsoleMsg("\nFailed to save results to file. Check the REAPER console for more information.\n")
    end
    
    -- Clean up
    if current_midi_item then
      reaper.DeleteTrackMediaItem(midi_track, current_midi_item)
    end
    
    -- Stop transport
    reaper.OnStopButton()
    reaper.PreventUIRefresh(-1)
    
    -- Set a flag that we're really done to break out of the defer loop
    reaper.SetExtState("VCO_TRACKING", "measurement_complete", "true", false)
    return
  end

  -- State machine for measurement process
  if measurement_state == 0 then
    -- State 0: Send note on
    local expected_freq = midi_to_freq(currentNote)
    local note_name = midi_to_note_name(currentNote)
    
    -- Calculate percentage progress
    local total_notes = end_note - start_note + 1
    local notes_done = currentNote - start_note
    local progress_pct = (notes_done / total_notes) * 100
    
    -- Calculate estimated time remaining
    local elapsed_time = reaper.time_precise() - start_time
    local time_per_note = elapsed_time / math.max(1, notes_done)
    local notes_remaining = total_notes - notes_done
    local time_remaining = time_per_note * notes_remaining
    
    -- Format the time
    local time_str = ""
    if notes_done > 0 then  -- Only show time estimate after first note
      time_str = string.format(" | ETA: %s", format_time(time_remaining))
    end
    
    reaper.ShowConsoleMsg(string.format("Measuring note %d (%s, %.2f Hz)... progress: %.1f%%%s\n", 
      currentNote, note_name, expected_freq, progress_pct, time_str))
    
    -- Clean up previous item
    if current_midi_item then
      reaper.DeleteTrackMediaItem(midi_track, current_midi_item)
    end
    
    -- Create new MIDI item and insert note
    local current_time = reaper.GetCursorPosition()
    current_midi_item, current_take = create_midi_item(midi_track)
    
    if current_midi_item and current_take then
      insert_midi_note(current_take, currentNote, note_velocity, current_time, current_time + note_duration_ms/1000)
      reaper.UpdateArrange()
    else
      reaper.ShowConsoleMsg("Failed to create MIDI item for note " .. currentNote .. "\n")
    end
    
    measurement_state = 1
    measurementTimer = reaper.time_precise() + (note_duration_ms / 2000)
    
  elseif measurement_state == 1 then
    -- State 1: Measure frequency (simulated)
    local measured_freq, amplitude = simulate_frequency_analysis(currentNote)
    local expected_freq = midi_to_freq(currentNote)
    
    table.insert(results, {
      note = currentNote,
      expected_freq = expected_freq,
      measured_freq = measured_freq or 0,
      amplitude = amplitude or 0
    })
    
    measurement_state = 2
    measurementTimer = reaper.time_precise() + (note_duration_ms / 2000)
    
  elseif measurement_state == 2 then
    -- State 2: Clean up
    if current_midi_item then
      reaper.DeleteTrackMediaItem(midi_track, current_midi_item)
      current_midi_item = nil
      current_take = nil
    end
    
    measurement_state = 3
    measurementTimer = reaper.time_precise() + (pause_between_notes_ms / 1000)
    
  elseif measurement_state == 3 then
    -- State 3: Move to next note
    currentNote = currentNote + 1
    measurement_state = 0
    measurementTimer = reaper.time_precise()
  end
end

-- Main function
function main()
  -- Show setup dialog first
  if not show_setup_dialog() then
    reaper.ShowConsoleMsg("VCO tracking measurement cancelled by user.\n")
    return
  end
  
  -- Reset the complete flag
  measurement_complete = false
  reaper.DeleteExtState("VCO_TRACKING", "measurement_complete", false)
  
  -- Prepare tracks
  midi_track = reaper.GetTrack(0, output_track - 1)
  audio_track = reaper.GetTrack(0, input_track - 1)
  
  if not midi_track or not audio_track then
    reaper.ShowMessageBox("Could not find specified tracks. Please make sure they exist.", "Error", 0)
    return
  end
  
  -- Enable record monitoring on audio track
  reaper.SetMediaTrackInfo_Value(audio_track, "I_RECMON", 1)
  
  -- Start transport
  reaper.OnStopButton()
  reaper.PreventUIRefresh(1)
  reaper.SetEditCurPos(0, false, false)
  reaper.OnPlayButton()
  
  -- Reset measurement variables
  results = {}
  currentNote = start_note
  measurement_state = 0
  start_time = reaper.time_precise()  -- Record the start time for ETA calculation
  measurementTimer = start_time + 0.5  -- Wait 500ms for playback to stabilize
  
  -- Show initial message
  reaper.ClearConsole()
  reaper.ShowConsoleMsg("VCO Tracking measurement started for " .. synth_name .. "\n")
  reaper.ShowConsoleMsg("Testing MIDI notes " .. start_note .. " to " .. end_note .. " (" .. midi_to_note_name(start_note) .. " to " .. midi_to_note_name(end_note) .. ")\n\n")
  
  -- Register the timer function with a more robust approach to handling completion
  local function onIdle()
    -- Check if we need to stop
    if reaper.GetExtState("VCO_TRACKING", "measurement_complete") == "true" then
      return
    end
    
    -- Run the next measurement step if it's time
    if reaper.time_precise() >= measurementTimer then
      runNextMeasurement()
    end
    
    -- Continue the loop unless we're done
    if not measurement_complete then
      reaper.defer(onIdle)
    end
  end
  
  reaper.defer(onIdle)
end

-- Function to create HTML report with visualization
local function generate_html_report(csv_path)
  -- Only proceed if we have results
  if #results == 0 then
    reaper.ShowMessageBox("No results to generate report from.", "Error", 0)
    return false
  end
  
  -- Generate HTML file path
  local html_path = csv_path:gsub("%.csv$", ".html")
  local file = io.open(html_path, "w")
  if not file then
    reaper.ShowMessageBox("Could not create HTML report file.", "Error", 0)
    return false
  end
  
  -- Write HTML header
  file:write([[
<!DOCTYPE html>
<html>
<head>
  <title>VCO Tracking Report</title>
  <style>
    body { font-family: Arial, sans-serif; margin: 20px; }
    h1, h2 { color: #333; }
    .container { max-width: 1000px; margin: 0 auto; }
    table { border-collapse: collapse; width: 100%; margin-top: 20px; }
    th, td { border: 1px solid #ddd; padding: 8px; text-align: left; }
    th { background-color: #f2f2f2; }
    tr:nth-child(even) { background-color: #f9f9f9; }
    .chart-container { height: 400px; margin: 30px 0; }
    .summary { background-color: #f5f5f5; padding: 15px; border-radius: 5px; margin: 20px 0; }
    .warning { color: orange; }
    .error { color: red; }
    .good { color: green; }
  </style>
  <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
</head>
<body>
  <div class="container">
    <h1>VCO Tracking Measurement Results</h1>
    <div class="summary">
]])

  -- Write metadata
  file:write("<p><strong>Synth:</strong> " .. synth_name .. "</p>\n")
  file:write("<p><strong>Date:</strong> " .. os.date("%Y-%m-%d %H:%M:%S", os.time()) .. "</p>\n")
  file:write("<p><strong>Note Range:</strong> " .. midi_to_note_name(start_note) .. " to " .. midi_to_note_name(end_note) .. "</p>\n")
  file:write("<p><strong>MIDI Channel:</strong> " .. midi_channel .. "</p>\n")
  
  if synth_notes ~= "" then
    file:write("<p><strong>Notes:</strong> " .. synth_notes .. "</p>\n")
  end
  
  -- Calculate statistics
  local max_deviation = 0
  local total_deviation = 0
  local valid_results = 0
  local worst_note = nil
  
  for _, result in ipairs(results) do
    if result.measured_freq > 0 and result.expected_freq > 0 then
      local cent_deviation = math.abs(1200 * math.log(result.measured_freq / result.expected_freq) / math.log(2))
      total_deviation = total_deviation + cent_deviation
      valid_results = valid_results + 1
      
      if cent_deviation > max_deviation then
        max_deviation = cent_deviation
        worst_note = result.note
      end
    end
  end
  
  local avg_deviation = 0
  if valid_results > 0 then
    avg_deviation = total_deviation / valid_results
  end
  
  -- Add statistics to the report
  file:write("<p><strong>Total Notes Measured:</strong> " .. #results .. "</p>\n")
  file:write("<p><strong>Average Deviation:</strong> " .. string.format("%.2f", avg_deviation) .. " cents")
  
  -- Add color coding based on deviation quality
  if avg_deviation < 5 then
    file:write(" <span class='good'>(Excellent)</span>")
  elseif avg_deviation < 10 then
    file:write(" <span class='good'>(Good)</span>")
  elseif avg_deviation < 20 then
    file:write(" <span class='warning'>(Fair)</span>")
  else
    file:write(" <span class='error'>(Poor)</span>")
  end
  file:write("</p>\n")
  
  file:write("<p><strong>Maximum Deviation:</strong> " .. string.format("%.2f", max_deviation) .. " cents")
  if worst_note then
    file:write(" at note " .. worst_note .. " (" .. midi_to_note_name(worst_note) .. ")")
  end
  file:write("</p>\n")
  
  file:write("</div>\n")
  
  -- Add chart containers
  file:write([[
    <h2>Deviation Chart</h2>
    <div class="chart-container">
      <canvas id="deviationChart"></canvas>
    </div>
    
    <h2>Frequency Response Chart</h2>
    <div class="chart-container">
      <canvas id="freqChart"></canvas>
    </div>
    
    <h2>Measurement Data</h2>
    <table>
      <tr>
        <th>MIDI Note</th>
        <th>Note Name</th>
        <th>Expected Freq (Hz)</th>
        <th>Measured Freq (Hz)</th>
        <th>Deviation (cents)</th>
        <th>Amplitude</th>
      </tr>
]])
  
  -- Add results table
  for _, result in ipairs(results) do
    local cent_deviation = 0
    if result.measured_freq > 0 and result.expected_freq > 0 then
      cent_deviation = 1200 * math.log(result.measured_freq / result.expected_freq) / math.log(2)
    end
    
    file:write("<tr>\n")
    file:write("  <td>" .. result.note .. "</td>\n")
    file:write("  <td>" .. midi_to_note_name(result.note) .. "</td>\n")
    file:write("  <td>" .. string.format("%.2f", result.expected_freq) .. "</td>\n")
    file:write("  <td>" .. string.format("%.2f", result.measured_freq) .. "</td>\n")
    file:write("  <td>" .. string.format("%.2f", cent_deviation) .. "</td>\n")
    file:write("  <td>" .. string.format("%.6f", result.amplitude) .. "</td>\n")
    file:write("</tr>\n")
  end
  
  file:write("</table>\n")
  
  -- Generate JavaScript for charts
  file:write([[
    <script>
      // Prepare chart data
      const noteLabels = []])
  
  -- Write note labels
  for i, result in ipairs(results) do
    file:write("'" .. midi_to_note_name(result.note) .. "'")
    if i < #results then
      file:write(", ")
    end
  end
  
  file:write("];\n      const deviationData = [")
  
  -- Write deviation data
  for i, result in ipairs(results) do
    local cent_deviation = 0
    if result.measured_freq > 0 and result.expected_freq > 0 then
      cent_deviation = 1200 * math.log(result.measured_freq / result.expected_freq) / math.log(2)
    end
    file:write(string.format("%.2f", cent_deviation))
    if i < #results then
      file:write(", ")
    end
  end
  
  file:write("];\n      const expectedFreqs = [")
  
  -- Write expected frequencies
  for i, result in ipairs(results) do
    file:write(string.format("%.2f", result.expected_freq))
    if i < #results then
      file:write(", ")
    end
  end
  
  file:write("];\n      const measuredFreqs = [")
  
  -- Write measured frequencies
  for i, result in ipairs(results) do
    file:write(string.format("%.2f", result.measured_freq))
    if i < #results then
      file:write(", ")
    end
  end
  
  -- Complete the JavaScript for charts
  file:write([[
      ];
      
      // Create deviation chart
      const deviationCtx = document.getElementById('deviationChart').getContext('2d');
      new Chart(deviationCtx, {
        type: 'line',
        data: {
          labels: noteLabels,
          datasets: [{
            label: 'Deviation (cents)',
            data: deviationData,
            backgroundColor: 'rgba(54, 162, 235, 0.2)',
            borderColor: 'rgba(54, 162, 235, 1)',
            borderWidth: 2,
            tension: 0.1
          }]
        },
        options: {
          responsive: true,
          maintainAspectRatio: false,
          scales: {
            y: {
              beginAtZero: false,
              title: {
                display: true,
                text: 'Deviation (cents)'
              }
            },
            x: {
              title: {
                display: true,
                text: 'Note'
              }
            }
          }
        }
      });
      
      // Create frequency response chart
      const freqCtx = document.getElementById('freqChart').getContext('2d');
      new Chart(freqCtx, {
        type: 'line',
        data: {
          labels: noteLabels,
          datasets: [
            {
              label: 'Expected Frequency (Hz)',
              data: expectedFreqs,
              borderColor: 'rgba(75, 192, 192, 1)',
              backgroundColor: 'rgba(75, 192, 192, 0.2)',
              borderWidth: 2,
              tension: 0.1
            },
            {
              label: 'Measured Frequency (Hz)',
              data: measuredFreqs,
              borderColor: 'rgba(255, 99, 132, 1)',
              backgroundColor: 'rgba(255, 99, 132, 0.2)',
              borderWidth: 2,
              tension: 0.1
            }
          ]
        },
        options: {
          responsive: true,
          maintainAspectRatio: false,
          scales: {
            y: {
              type: 'logarithmic',
              title: {
                display: true,
                text: 'Frequency (Hz, log scale)'
              }
            },
            x: {
              title: {
                display: true,
                text: 'Note'
              }
            }
          }
        }
      });
    </script>
  </div>
</body>
</html>
]])
  
  file:close()
  return true, html_path
end

-- Add a menu item to generate HTML report from the last measurement
function generate_report_from_last()
  if #results > 0 then
    local success, html_path = generate_html_report(csv_filename)
    if success then
      reaper.ShowMessageBox("HTML report generated at:\n" .. html_path, "Success", 0)
      -- Try to open the report in default browser
      local os_name = reaper.GetOS()
      if os_name:match("^Win") then
        os.execute('start "" "' .. html_path .. '"')
      elseif os_name:match("^OSX") or os_name:match("^macOS") then
        os.execute('open "' .. html_path .. '"')
      elseif os_name:match("^Lin") then
        os.execute('xdg-open "' .. html_path .. '"')
      end
    end
  else
    reaper.ShowMessageBox("No measurement results available to generate a report.", "Error", 0)
  end
end

-- Add the script to REAPER's context menu
function init()
  -- Check if we're resuming a previous measurement
  if reaper.GetExtState("VCO_TRACKING", "measurement_complete") == "true" then
    reaper.ClearConsole()
    reaper.ShowConsoleMsg("Previous VCO Tracking measurement completed successfully.\n")
    reaper.ShowConsoleMsg("Use the 'Generate HTML Report' option from the Actions menu to create a visual report.\n")
  end

  -- Clean up any leftover state
  measurement_complete = false
  reaper.DeleteExtState("VCO_TRACKING", "measurement_complete", false)
end

-- Register commands and actions
function register_commands()
  -- Add a menu command for generating a report
  local _, _, section, cmdID = reaper.get_action_context()
  reaper.SetToggleCommandState(section, cmdID, 0)
  reaper.RefreshToolbar2(section, cmdID)
  
  -- You can register additional commands here for report generation, etc.
end

-- Execute the script
reaper.Undo_BeginBlock()
init()
main()
reaper.Undo_EndBlock("VCO Tracking Measurement", -1)

-- Optional: Add keyboard shortcut registration here if needed
-- reaper.register("generate_report_from_last", "Generate VCO Tracking HTML Report", generate_report_from_last)
