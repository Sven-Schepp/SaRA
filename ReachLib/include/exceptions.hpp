/*
This file is part of SaRA.

SaRA is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
TUM, either version 3 of the License, or
(at your option) any later version.

SaRA is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details: https://www.gnu.org/licenses/.
*/

#include <exception>
#include <string>

#ifndef EXCEPTIONS_H
#define EXCEPTIONS_H

struct PredictionNotSupportedException : public std::exception {
  const char* what() const throw() {
    return "The prediction for this model is not supported as it requires a predicted velocity.";
  }
};

#endif // EXCEPTIONS_H