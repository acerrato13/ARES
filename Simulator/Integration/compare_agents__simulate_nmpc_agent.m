classdef compare_agents__simulate_nmpc_agent < matlab.System
% Python def simulate_nmpc_agent imported from python module compare_agents.py
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
    py.importlib.import_module('compare_agents');
end
end
function [y] = stepImpl(obj,agent,initial_state,target_state,steps)
    y = double(py.compare_agents.simulate_nmpc_agent(agent,initial_state,target_state,steps));
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
