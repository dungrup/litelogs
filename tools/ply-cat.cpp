/* The copyright in this software is being made available under the BSD
 * Licence, included below.  This software may be subject to other third
 * party and contributor rights, including patent rights, and no such
 * rights are granted under this licence.
 *
 * Copyright (c) 2019, ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of the ISO/IEC nor the names of its contributors
 *   may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Concatenate the pedestrian/cyclist cloud that litelogs routed around the
// codec with the decoded remainder, producing one ply per frame.
//
// Both inputs are already written in the same external coordinate system
// (metres, velodyne frame): SequenceCodec::writeOutputFrame has applied
// outputScale/outputOrigin to the recon, and compressOneFrame wrote the ped
// cloud through ply::write with a scale of 1/inputScale.  So there is no
// transform to apply here -- only a read, an append and a write.

#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "PCCPointSet.h"
#include "ply.h"
#include "program_options_lite.h"
#include "version.h"

using namespace std;
using namespace pcc;

namespace fs = std::filesystem;

//============================================================================

struct Options {
  // directory of decoded remainder clouds, <stem>.ply
  std::string reconDir;

  // directory of extracted pedestrian clouds, <stem>/ped.ply
  std::string pedDir;

  // directory to write the merged <stem>.ply into
  std::string outDir;

  // file listing the stems to process, one per line
  std::string stemList;

  // per-frame summary csv (optional)
  std::string csvPath;

  // scale applied by ply::read before truncation to int32, and inverted on
  // write.  1000 keeps the millimetre precision of the ped cloud.
  double positionScale;

  // output mode for ply writing (binary or ascii)
  bool outputBinaryPly;
};

//============================================================================

static bool
parseParameters(int argc, char* argv[], Options& params)
{
  namespace po = df::program_options_lite;
  bool print_help = false;

  /* clang-format off */
  // The definition of the program/config options, along with default values.
  //
  // NB: when updating the following tables:
  //      (a) please keep to 80-columns for easier reading at a glance,
  //      (b) do not vertically align values -- it breaks quickly
  //
  po::Options opts;
  opts.addOptions()
  ("help", print_help, false, "this help text")
  ("config,c", po::parseConfigFile, "configuration file name")

  ("reconDir",
    params.reconDir, {},
    "Directory of decoded remainder clouds, named <stem>.ply")

  ("pedDir",
    params.pedDir, {},
    "Directory of extracted pedestrian clouds, named <stem>/ped.ply")

  ("outDir",
    params.outDir, {},
    "Directory to write the merged <stem>.ply into")

  ("stemList",
    params.stemList, {},
    "File listing the frame stems to process, one per line")

  ("csvPath",
    params.csvPath, {},
    "Optional per-frame summary csv")

  ("positionScale",
    params.positionScale, 1000.,
    "Scale applied on read before truncation to int32, inverted on write.\n"
    "1000 preserves the millimetre precision of the ped cloud")

  ("outputBinaryPly",
    params.outputBinaryPly, false,
    "Output ply files using binary (or otherwise ascii) format")
  ;
  /* clang-format on */

  po::setDefaults(opts);
  po::ErrorReporter err;
  const list<const char*>& argv_unhandled =
    po::scanArgv(opts, argc, (const char**)argv, err);

  for (const auto arg : argv_unhandled) {
    err.warn() << "Unhandled argument ignored: " << arg << "\n";
  }

  if (argc == 1 || print_help) {
    po::doHelp(std::cout, opts, 78);
    return false;
  }

  if (params.reconDir.empty())
    err.error() << "reconDir not set\n";
  if (params.pedDir.empty())
    err.error() << "pedDir not set\n";
  if (params.outDir.empty())
    err.error() << "outDir not set\n";
  if (params.stemList.empty())
    err.error() << "stemList not set\n";
  if (params.positionScale <= 0.)
    err.error() << "positionScale must be positive\n";

  po::dumpCfg(cout, opts, 4);

  return !err.is_errored;
}

//---------------------------------------------------------------------------
// Read the work list.  Frames are processed in the order given: unlike a
// directory scan the file already fixes an order, so there is nothing to sort.

static bool
readStemList(const std::string& path, std::vector<std::string>& stems)
{
  std::ifstream in(path);
  if (!in.is_open()) {
    cout << "Error: can't open stem list: " << path << endl;
    return false;
  }

  std::string line;
  while (std::getline(in, line)) {
    const auto first = line.find_first_not_of(" \t\r\n");
    if (first == std::string::npos)
      continue;
    const auto last = line.find_last_not_of(" \t\r\n");
    stems.push_back(line.substr(first, last - first + 1));
  }

  return true;
}

//---------------------------------------------------------------------------
// PCCPointSet3::append copies an attribute only when both clouds carry it,
// and silently drops it otherwise.  Refuse the merge rather than emit a cloud
// whose ped points hold uninitialised attribute values.

static bool
attributesMatch(const PCCPointSet3& a, const PCCPointSet3& b)
{
  return a.hasColors() == b.hasColors()
    && a.hasReflectances() == b.hasReflectances()
    && a.hasLaserAngles() == b.hasLaserAngles();
}

//============================================================================

int
main(int argc, char* argv[])
{
  cout << "MPEG PCC ply concatenation tool from Test Model C13" << endl;
  cout << "  litelogs fork, version " << ::pcc::version << endl;

  Options opts;
  if (!parseParameters(argc, argv, opts))
    return 1;

  std::vector<std::string> stems;
  if (!readStemList(opts.stemList, stems))
    return 1;

  ply::PropertyNameMap propNames;
  propNames.position = {"x", "y", "z"};

  std::ofstream csv;
  if (!opts.csvPath.empty()) {
    csv.open(opts.csvPath, std::ios::trunc);
    if (!csv.is_open()) {
      cout << "Error: can't open csv: " << opts.csvPath << endl;
      return 1;
    }
    csv << "stem,recon_points,ped_points,total_points,status\n";
  }

  int okCount = 0;
  int noPedCount = 0;
  int failCount = 0;

  try {
    fs::create_directories(opts.outDir);

    for (const auto& stem : stems) {
      const auto reconPath = fs::path(opts.reconDir) / (stem + ".ply");
      const auto pedPath = fs::path(opts.pedDir) / stem / "ped.ply";
      const auto outPath = fs::path(opts.outDir) / (stem + ".ply");

      std::string status = "ok";
      size_t reconPoints = 0;
      size_t pedPoints = 0;

      PCCPointSet3 outCloud;
      if (
        !ply::read(reconPath.string(), propNames, opts.positionScale, outCloud)
        || outCloud.getPointCount() == 0) {
        cout << "Error: can't read recon: " << reconPath << endl;
        status = "recon_failed";
      } else {
        reconPoints = outCloud.getPointCount();

        // A frame whose detections held no pedestrians has no ped.ply at all;
        // that is normal, and the recon simply passes through unchanged.
        if (!fs::exists(pedPath)) {
          status = "ok_no_ped";
        } else {
          PCCPointSet3 pedCloud;
          if (!ply::read(
                pedPath.string(), propNames, opts.positionScale, pedCloud)) {
            cout << "Error: can't read ped: " << pedPath << endl;
            status = "ped_failed";
          } else if (!attributesMatch(outCloud, pedCloud)) {
            cout << "Error: attribute mismatch: " << pedPath << endl;
            status = "attr_mismatch";
          } else {
            pedPoints = pedCloud.getPointCount();
            outCloud.append(pedCloud);
          }
        }
      }

      const bool readable = status == "ok" || status == "ok_no_ped";

      if (readable) {
        if (!ply::write(
              outCloud, propNames, 1. / opts.positionScale, Vec3<double>(0.),
              outPath.string(), !opts.outputBinaryPly)) {
          cout << "Error: can't write output: " << outPath << endl;
          status = "write_failed";
        }
      }

      if (status == "ok")
        okCount++;
      else if (status == "ok_no_ped")
        noPedCount++;
      else
        failCount++;

      cout << stem << ": " << reconPoints << " + " << pedPoints << " = "
           << (readable ? outCloud.getPointCount() : 0) << " (" << status
           << ")" << endl;

      if (csv.is_open())
        csv << stem << ',' << reconPoints << ',' << pedPoints << ','
            << (readable ? outCloud.getPointCount() : 0) << ',' << status
            << '\n';
    }
  }
  catch (const exception& e) {
    cerr << "Error: " << e.what() << endl;
    return 1;
  }

  cout << "Processed " << stems.size() << " stems: merged=" << okCount
       << ", no ped=" << noPedCount << ", failed=" << failCount << endl;

  return failCount ? 1 : 0;
}
