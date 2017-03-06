classdef PiBot < handle
 
    properties(Access = public)
        TCP_MOTORS;
        TCP_CAMERA;
    end
 
    properties (Access = private, Constant)
        TIMEOUT = 10;
 
        PORT_MOTORS = 43900; % some random ports that should be unused as they are above 2000?
        PORT_CAMERAS = 43901;
 
        IMAGE_WIDTH = 640/2;
        IMAGE_HEIGHT = 480/2;
        IMAGE_SIZE = PiBot.IMAGE_WIDTH * PiBot.IMAGE_HEIGHT * 3;
 
        FN_ARG_SEPARATOR = ','
        FN_GET_IMAGE = 'getImageFromCamera'
        FN_MOTOR_SPEEDS = 'setMotorSpeeds'
        FN_MOTOR_TICKS = 'getMotorTicks'
        FN_DISPLAY_VALUE = 'setDisplayValue'
        FN_DISPLAY_MODE = 'setDisplayMode'
    end
 
    methods
        function obj = PiBot(address)
 
            obj.TCP_MOTORS = tcpip(address, PiBot.PORT_MOTORS, 'NetworkRole', 'client', 'Timeout', PiBot.TIMEOUT);
            obj.TCP_CAMERA = tcpip(address, PiBot.PORT_CAMERAS, 'NetworkRole', 'client', 'Timeout', PiBot.TIMEOUT);
 
            % Configure the TCPIP objects
            %obj.TCP_CAMERA.Timeout = PiBot.TIMEOUT;
            %obj.TCP_MOTORS.Timeout = PiBot.TIMEOUT;
            obj.TCP_CAMERA.InputBufferSize = PiBot.IMAGE_SIZE;
 
        end
 
        function delete(obj)
            delete(obj.TCP_MOTORS);
            delete(obj.TCP_CAMERA);
        end
 
        function imgVect = getVectorFromCamera(obj)
            fopen(obj.TCP_CAMERA);
            fprintf(obj.TCP_CAMERA, [PiBot.FN_GET_IMAGE PiBot.FN_ARG_SEPARATOR '100']); 
            imgVect = fread(obj.TCP_CAMERA, PiBot.IMAGE_SIZE, 'uint8')./255;
            fclose(obj.TCP_CAMERA);
        end
         
        function img = getImageFromCamera(obj)
            % Attempt to retrieve the image
            try
                vector = obj.getVectorFromCamera();
            catch error
                fprintf('Error: retrieving image (''%s'')! Empty image array returned!\n', error.message);
                img = [];
                return;
            end
 
            % Convert the image to MATLAB format (if it's the correct size)
            if length(vector) == PiBot.IMAGE_SIZE
                img = PiBot.vect2img(vector);
            else
                fprintf('Error: Size of data received (%d) did not match expected image size (%d). Empty image array returned!\n', length(vector), PiBot.IMAGE_SIZE);
                img = [];
            end
        end
         
        function setMotorSpeeds(obj, motorA, motorB)
            data = [PiBot.FN_MOTOR_SPEEDS];
            data = [data PiBot.FN_ARG_SEPARATOR num2str(motorA) PiBot.FN_ARG_SEPARATOR num2str(motorB)]
             
            fopen(obj.TCP_MOTORS);
            fprintf(obj.TCP_MOTORS, data);
            fclose(obj.TCP_MOTORS);
        end
 
        function ticks = getMotorTicks(obj)
            data = [PiBot.FN_MOTOR_TICKS,PiBot.FN_ARG_SEPARATOR 'A']; % needed for the Pi code
            fopen(obj.TCP_MOTORS);
            fprintf(obj.TCP_MOTORS, data);
%            iter = 0;
            % We don't know the size of the file so...
%             c='';
%             s='';
%             while ~strcmp(char(c),'\n')
%                 c = ( fread(obj.TCP_MOTORS,1,'char') );
%                 s=[s c];
%                 iter = iter + 1;
%                 if iter > 30 % the tick array should never be this large, which means unsuccessful read...
%                     disp('Tick retreval timeout! (PiBot.m ~line 103) returning NULL tick value ...');
%                     pause(0.1);
%                     ticks = [];
%                     fclose(obj.TCP_MOTORS);
%                     disp('Turning all motors OFF') % this is a precaution, you can comment out this if you want
%                     setMotorSpeeds(['A','B','C','D'], [0,0,0,0]);
%                     return;
%                 end
%             end

            s = fgetl(obj.TCP_MOTORS);
            fclose(obj.TCP_MOTORS);

            % Convert ticks to numerical array
%             ticks = sscanf(s,'%f',inf);
            ticks = sscanf(s,'%d');

        end

        function setDisplayValue(obj, val)
        %PiBot.setDisplayValue  Write to the robot display
        %
        % PB.setDisplayValue(V) writes the value of V to the robot's display, using
        % the current mode.  The range of allowable values depends on the mode:
        %  - hexadecimal 0 to 255
        %  - unsigned decimal 0 to 99
        %  - signed decimal -9 to 9
        %
        % See also PiBot.setDisplayMode.
            data = [PiBot.FN_DISPLAY_VALUE];
            data = [data PiBot.FN_ARG_SEPARATOR num2str(val)]
            fprintf('send: [%s]\n', data);
             
            fopen(obj.TCP_MOTORS);
            fprintf(obj.TCP_MOTORS, data);
            fclose(obj.TCP_MOTORS);
        end
 
        function setDisplayMode(obj, val)
        %PiBot.setDisplayMode  Set the robot display mode
        %
        % PB.setDisplayMode(M) sets the numerical mode for the robot's display:
        %  - 'x' hexadecimal
        %  - 'u' unsigned decimal
        %  - 'd' signed decimal -9 to 9
        %
        % In decimal modes the decimal point on the right-hand digit is lit.
        %
        % See also PiBot.setDisplayValue.
            data = [PiBot.FN_DISPLAY_MODE];
            data = [data PiBot.FN_ARG_SEPARATOR val]
            fprintf('send: [%s]\n', data);
             
            fopen(obj.TCP_MOTORS);
            fprintf(obj.TCP_MOTORS, data);
            fclose(obj.TCP_MOTORS);
        end
 
    end
     
    methods(Static)
        function img = vect2img(data)
%             img = zeros(PiBot.IMAGE_WIDTH, PiBot.IMAGE_HEIGHT, 3);
%             it = 1;
             
            img = reshape([data(1:3:end); data(2:3:end); data(3:3:end)],PiBot.IMAGE_WIDTH, PiBot.IMAGE_HEIGHT, 3);
%             tic;
%             for jj = 1:size(img,2) % loop through columns
%                 for ii = 1:size(img,1) % loop through rows
%                     for kk = 1:size(img,3)
%                         img(ii,jj,kk) = data(it); 
%                         it = it + 1;
%                     end
%                 end
%             end
%             toc;
%             disp('time spend in vect2mat function');
        end        
    end
end
