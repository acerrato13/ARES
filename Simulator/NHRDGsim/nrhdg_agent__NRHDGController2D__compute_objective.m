classdef nrhdg_agent__NRHDGController2D__compute_objective < matlab.System
% Python method compute_objective imported from python class NRHDGController2D in python module nrhdg_agent.py
properties
    role_switcher (1,1) double {mustBeReal}
    config (1,1) double {mustBeReal}
end
properties (Access = private)
    self
end
methods(Access = protected)
function validateInputsImpl(obj, varargin)
if ~isempty(varargin{1})
    validateattributes(varargin{1}, {'double'}, {'size',[1 1]});
end
if ~isempty(varargin{2})
    validateattributes(varargin{2}, {'double'}, {'size',[1 1]});
end
if ~isempty(varargin{3})
    validateattributes(varargin{3}, {'double'}, {'size',[1 1]});
end
if ~isempty(varargin{4})
    validateattributes(varargin{4}, {'double'}, {'size',[1 1]});
end
end
function setupImpl(obj)
if coder.target('MATLAB')
    py.importlib.import_module('nrhdg_agent');
    obj.self = py.nrhdg_agent.NRHDGController2D(obj.role_switcher,obj.config);
end
end
function [y] = stepImpl(obj,ego_state,opp_state,u_sequence,v_sequence)
if coder.target('MATLAB')
    y = double(obj.self.compute_objective(ego_state,opp_state,u_sequence,v_sequence));
else
    y = double(pyClass.compute_objective(ego_state,opp_state,u_sequence,v_sequence));
end
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
