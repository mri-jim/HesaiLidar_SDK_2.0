// Record PCAP from network-connected Hesai lidar for N seconds (default 60)
// Usage: ./record_pcap <config.ini> [seconds]
#include "hesai_lidar_sdk.hpp"
#include "../config/driver_sample_config.hpp"

#include <csignal>
#include <cstring>
#include <fstream>
#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>

using RecordPointType = LidarPointXYZICRT;

static volatile bool g_running = true;

void signalHandler(int) { g_running = false; }

// Minimal callback — just prints frame info so we know data is flowing
void lidarCallback(const LidarDecodedFrame<RecordPointType>& frame) {
  printf("frame:%d  pts:%u  pkts:%u  t:[%.3f - %.3f]\n",
         frame.frame_index, frame.points_num, frame.packet_num,
         frame.frame_start_timestamp, frame.frame_end_timestamp);
}

void faultMessageCallback(const FaultMessageInfo&) {}

static std::string makeTimestampFilename() {
  auto now = std::chrono::system_clock::now();
  auto t = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  localtime_r(&t, &tm);
  std::ostringstream oss;
  oss << "Hesai_recording_" << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".pcap";
  return oss.str();
}

int main(int argc, char* argv[]) {
  if (argc < 2) {
    fprintf(stderr, "Usage: %s <config.ini> [seconds=60] [output.pcap]\n", argv[0]);
    return 1;
  }

#ifndef _MSC_VER
  // Increase UDP receive buffer
  if (system("sudo sh -c \"echo 562144000 > /proc/sys/net/core/rmem_max\"") == -1) {
    printf("Warning: could not set rmem_max\n");
  }
#endif

  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);

  // Parse args
  const char* config_path = argv[1];
  int record_seconds = 60;
  std::string output_pcap;

  if (argc >= 3) record_seconds = atoi(argv[2]);
  if (argc >= 4) output_pcap = argv[3];
  if (output_pcap.empty()) output_pcap = makeTimestampFilename();

  // Load config
  std::ifstream probe(config_path);
  if (!probe.good()) {
    fprintf(stderr, "Error: config file not found: %s\n", config_path);
    return 1;
  }
  probe.close();

  std::unordered_map<std::string, std::string> kv;
  std::string err;
  if (!hesai::lidar::sample_config::LoadIniMap(config_path, &kv, &err)) {
    fprintf(stderr, "Config error: %s\n", err.c_str());
    return 1;
  }
  if (!hesai::lidar::sample_config::MergeSourceTypeProfileIntoInput(&kv, &err)) {
    fprintf(stderr, "Config error: %s\n", err.c_str());
    return 1;
  }

  DriverParam param;
  if (!hesai::lidar::sample_config::ApplyToDriverParam(kv, "", &param, &err)) {
    fprintf(stderr, "Config error: %s\n", err.c_str());
    return 1;
  }

  if (param.decoder_param.socket_buffer_size == 0) {
    param.decoder_param.socket_buffer_size = 262144000;
  }

  // Init SDK
  HesaiLidarSdk<RecordPointType> sdk;
  sdk.Init(param);
  sdk.RegRecvCallback(lidarCallback);
  sdk.RegRecvCallback(faultMessageCallback);

  sdk.Start();
  if (sdk.lidar_ptr_->GetInitFinish(FailInit)) {
    fprintf(stderr, "SDK init failed\n");
    sdk.Stop();
    return 1;
  }

  // Auto-save correction file from lidar via PTC
  std::string corr_file = "jt128_correction.csv";
  if (sdk.lidar_ptr_->SaveCorrectionFile(corr_file) == 0) {
    printf("=== Saved correction file: %s ===\n", corr_file.c_str());
  } else {
    printf("=== Warning: Could not save correction file via PTC ===\n");
  }

  // Start PCAP recording
  printf("=== Recording to: %s for %d seconds (Ctrl+C to stop early) ===\n",
         output_pcap.c_str(), record_seconds);

  sdk.lidar_ptr_->StartRecordPcap(output_pcap);

  auto start = std::chrono::steady_clock::now();
  while (g_running) {
    auto elapsed = std::chrono::steady_clock::now() - start;
    if (std::chrono::duration_cast<std::chrono::seconds>(elapsed).count() >= record_seconds) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Stop recording
  sdk.lidar_ptr_->StopRecordPcap();
  printf("=== Recording stopped. Saved: %s ===\n", output_pcap.c_str());

  sdk.Stop();
  return 0;
}
