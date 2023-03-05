classdef Controller
    properties
    type
    Kp = [100; 80];
    Kv = [30; 30];
    end
    
    methods
        function this = Controller(type)
            this.type = type;
        end  
        
        %扭矩
        function ControllerTorque = Torque(this, PCmd, P, VCmd, V, ACmd, M, C)
                switch this.type
                    case 'Linear'
                        % PD
                        ControllerTorque = this.Kp .* (PCmd - P) + ...
                                           this.Kv .* (VCmd - V);
                    case 'CTC'
                        % CTC
                        ControllerTorque = M * (this.Kp .* (PCmd - P) + ...
                                                this.Kv .* (VCmd - V) + ...
                                                ACmd) + ...
                                           C * VCmd;
                    case 'DFF'
                        % DFF
                        ControllerTorque = M * ACmd + ...
                                           C * VCmd + ...
                                           this.Kp .* (PCmd - P) + ...
                                           this.Kv .* (VCmd - V);          
                    case 'PD-like'
                        ControllerTorque = this.Kv .* (this.Kp .* (PCmd - P) - V);

                    case "CBF"
                        
                        ControllerTorque = 1;
                    
                end
        end
        
    end
    
end
    