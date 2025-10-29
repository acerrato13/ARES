classdef nrhdg_agent__NRHDGController2D__solve_saddle_point < matlab.System
% Python method solve_saddle_point imported from python class NRHDGController2D in python module nrhdg_agent.py
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
end
function setupImpl(obj)
if coder.target('MATLAB')
    py.importlib.import_module('nrhdg_agent');
    obj.self = py.nrhdg_agent.NRHDGController2D(obj.role_switcher,obj.config);
end
end
function [u_seq,v_seq] = stepImpl(obj,ego_state,opp_state)
    if coder.target('MATLAB')
        ytmp = cell(obj.self.solve_saddle_point(ego_state,opp_state));
        u_seq = double(ytmp{1});
        v_seq = double(ytmp{2});
    else
        ytmp = pyClass.solve_saddle_point(ego_state,opp_state);
        u_seq = double(zeros(1,1));
        v_seq = double(zeros(1,1));
    end
end
function varargout = getOutputDataTypeImpl(obj)
    varargout{1} = 'double';
    varargout{2} = 'double';
end
function varargout = getOutputSizeImpl(obj)
    varargout{1} = [1 1];
    varargout{2} = [1 1];
end
function varargout = isOutputComplexImpl(obj)
    varargout{1} = false;
    varargout{2} = false;
end
function varargout = isOutputFixedSizeImpl(obj)
    varargout{1} = true;
    varargout{2} = true;
end
end
methods(Static, Access = protected)
end
end
