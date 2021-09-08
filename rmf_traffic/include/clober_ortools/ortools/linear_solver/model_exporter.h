// Copyright 2010-2021 Google LLC
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OR_TOOLS_LINEAR_SOLVER_MODEL_EXPORTER_H_
#define OR_TOOLS_LINEAR_SOLVER_MODEL_EXPORTER_H_

#include <string>
#include <vector>

#include "clober_ortools/absl/status/statusor.h"
#include "clober_ortools/absl/strings/str_format.h"
#include "clober_ortools/ortools/base/hash.h"
#include "clober_ortools/ortools/base/macros.h"
#include "clober_ortools/ortools/linear_solver/linear_solver.pb.h"

namespace operations_research {

/// Export options.
struct MPModelExportOptions {
  MPModelExportOptions() {}

  /// Obfuscates variable and constraint names.
  bool obfuscate = false;
  /// Whether to log invalid variable and constraint names.
  bool log_invalid_names = false;

  /**
   * For .lp files only. Decides whether variables unused in the objective and
   * constraints are shown when exported to a file.
   */
  bool show_unused_variables = false;

  /**
   * For .lp files only. Maximum line length in exported files. The default
   * was chosen so that SCIP can read the files.
   */
  int max_line_length = 10000;
};

/**
 *  Outputs the current model (variables, constraints, objective) as a string
 * encoded in the so-called "CPLEX LP file format" as generated by SCIP.
 * The LP file format is easily readable by a human.
 *
 * Returns false if some error has occurred during execution.
 * The validity of names is automatically checked. If a variable name or a
 * constraint name is invalid or non-existent, a new valid name is
 * automatically generated.
 *
 * If 'obfuscated' is true, the variable and constraint names of proto_
 * are not used.  Variable and constraint names of the form "V12345"
 * and "C12345" are used instead.
 *
 * For more information about the different LP file formats:
 * http://lpsolve.sourceforge.net/5.5/lp-format.htm
 * The following give a reasonable idea of the CPLEX LP file format:
 * http://lpsolve.sourceforge.net/5.5/CPLEX-format.htm
 * http://tinyurl.com/cplex-lp-format
 * http://www.gurobi.com/documentation/5.1/reference-manual/node871
 */
absl::StatusOr<std::string> ExportModelAsLpFormat(
    const MPModelProto& model,
    const MPModelExportOptions& options = MPModelExportOptions());

/**
 * Outputs the current model (variables, constraints, objective) as a string
 * encoded in MPS file format, using the "free" MPS format.
 *
 * Returns false if some error has occurred during execution. Models with
 * maximization objectives trigger an error, because MPS can encode only
 * minimization problems.
 *
 * The validity of names is automatically checked. If a variable name or a
 * constraint name is invalid or non-existent, a new valid name is
 * automatically generated.
 *
 * Name validity and obfuscation works exactly as in ExportModelAsLpFormat().
 *
 * For more information about the MPS format:
 * http://en.wikipedia.org/wiki/MPS_(format)
 * A close-to-original description coming from OSL:
 * http://tinyurl.com/mps-format-by-osl
 * A recent description from CPLEX:
 * http://tinyurl.com/mps-format-by-cplex
 * CPLEX extensions:
 * http://tinyurl.com/mps-extensions-by-cplex
 * Gurobi's description:
 * http://www.gurobi.com/documentation/5.1/reference-manual/node869
 */
absl::StatusOr<std::string> ExportModelAsMpsFormat(
    const MPModelProto& model,
    const MPModelExportOptions& options = MPModelExportOptions());

}  // namespace operations_research

#endif  // OR_TOOLS_LINEAR_SOLVER_MODEL_EXPORTER_H_
