classdef Logger < handle

    properties
        logger_size
        logger_length
        log
        time
    end

    methods
        function obj = Logger(vector_size_, time_len_)
            obj.logger_size = vector_size_;
            obj.logger_length = time_len_;
            obj.log = NaN([obj.logger_size, obj.logger_length]);
            obj.time = NaN([1, obj.logger_length]);
        end

        function obj = update(obj, data, idx, time)
            obj.log(:, idx) = data;
            obj.time(idx) = time;
        end
    end
end