classdef nrhdg_agent__objective_u < matlab.System
% Python def objective_u imported from python module nrhdg_agent.py
methods(Access = protected)
function validateInputsImpl(obj, varargin)
if ~isempty(varargin{1})
    validateattributes(varargin{1}, {'double'}, {'size',[1 1]});
end
end
function setupImpl(obj)
if coder.target('MATLAB')
    py.importlib.import_module('nrhdg_agent');
end
end
function [y] = stepImpl(obj,u)
    y = double(py.nrhdg_agent.objective_u(u));
end
function varargout = getOutputDataTypeImpl(obj)
    varargout{1} = 'double';
end
function varargout = getOutputSizeImpl(obj)
    varargout{1} = [1 1];
end
function varargout = isOutputComplexImpl(obj)
    varargout{1} = false;
end
function varargout = isOutputFixedSizeImpl(obj)
    varargout{1} = true;
end
end
methods(Static, Access = protected)
end
end
