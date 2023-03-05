classdef Robot <handle
    properties
        axis = 2;
        L = [0.24, 0.24];
        c = [0.12, 0.12];
        m =[2, 2];
        I = [0.0267, 0.0267];
        sampT = 0.001;
        LB = [0, 0, 0, 0];
        UB = [0.1, 0.1, 500, 500];
        controller  % 控制器
        Ttype  % 創建軌跡型態
        CtrlP  % 控制點
        F_par  % 傅立葉係數    
        Theta % 鑑別出的參數
    end
    
    methods
        function this = Robot()
        end
        %% 設置控制器
        % 輸入 : 控制器類別
        % 輸出 : robot object
        function obj = setController(obj, type)
            obj.controller = Controller(type);
        end
        
        %% 設置鑑別參數
        % 輸入 : 鑑別參數
        % 輸出 : 更改變數
        function setTheta(this, theta)
            this.Theta = theta;
        end
        
        %% 設置軌跡型態 
        % 輸入 : 1.控制點或參數 2.軌跡生成型態
        % 輸出 : 更改變數
        function setTtype(this, input, type)
            this.Ttype = type;
            switch type
                case 'CtrlP'
                   this.CtrlP = input;
                case 'Fourier'
                   this.F_par = input;
            end
        end
        
        %% 軌跡規劃
        % 輸入 : 軌跡生成型態
        % 輸出 : struct P,V,A (角度,角速度,角加速度命令)
        function Output = Trajectory(this)
            switch this.Ttype
                % 控制點做軌跡
                case 'CtrlP' 
                    CtrlPnum = length(this.CtrlP(:, 1));  % 控制點數量
                    [rec.P, rec.V, rec.A] = deal([]);
                    for i = 1 : CtrlPnum - 1
                        % 控制點的時間
                        t0 = this.CtrlP(i, 1);  t1 = this.CtrlP(i+1, 1);
                        Des = [];
                        % 串接各軸的位置、速度、加速度
                        for j = 1 : this.axis
                            Des = [Des; 
                                   this.CtrlP(i, (j - 1)*3 + 2), this.CtrlP(i+1, (j - 1)*3 + 2), ...
                                   this.CtrlP(i, (j - 1)*3 + 3), this.CtrlP(i+1, (j - 1)*3 + 3), ...
                                   this.CtrlP(i, (j - 1)*3 + 4), this.CtrlP(i+1, (j - 1)*3 + 4)];
                        end

                        % 時間 (五階多項式)
                        Tcom = [t0^5, t1^5, 5*t0^4, 5*t1^4, 20*t0^3, 20*t1^3; ...
                                t0^4, t1^4, 4*t0^3, 4*t1^3, 12*t0^2, 12*t1^2; ...
                                t0^3, t1^3, 3*t0^2, 3*t1^2, 6*t0,    6*t1; ...
                                t0^2, t1^2, 2*t0,   2*t1,   2,       2; ...
                                t0,   t1,   1,      1,      0,       0; ...
                                1,    1,    0,      0,      0,       0;];
                        % 多項式係數
                        coef = Des/Tcom;

                        for t = t0 : this.sampT : t1 - this.sampT
                            rec.P = [rec.P; (coef*[t^5;     t^4;     t^3;    t^2;  t;  1])'];
                            rec.V = [rec.V; (coef*[5*t^4;   4*t^3;   3*t^2;  2*t;  1;  0])'];
                            rec.A = [rec.A; (coef*[20*t^3;  12*t^2;  6*t;    2;    0;  0])'];    
                        end
                    end
                    Output = this.Inverse_Kinematics(rec.P);
                    
                %傅立葉作軌跡    
                case 'Fourier'
                    tf = 12;
                    wf = (2 * pi)/tf ;
                    N = 5;    %用N組基底頻率
                    n = tf/this.sampT;   %軌跡點的個數
                    [rec.P, rec.V, rec.A] = deal(zeros(n ,2));
                    r = 1;
                    for t = this.sampT : this.sampT : tf
                        [p1, v1, a1, p2, v2, a2] = deal(0);
                        for i = 1 : N
                            p1 = p1 + (this.F_par(i, 1)*sin(wf*i*t) - this.F_par(i+5, 1)*cos(wf*i*t))/wf/i;
                            v1 = v1 + this.F_par(i, 1)*cos(wf*i*t) + this.F_par(i+5, 1)*sin(wf*i*t);
                            a1 = a1 + (-this.F_par(i, 1)*sin(wf*i*t) + this.F_par(i+5, 1)*cos(wf*i*t))*wf*i;

                            p2 = p2 + (this.F_par(i, 2)*sin(wf*i*t) - this.F_par(i+5, 2)*cos(wf*i*t))/wf/i;
                            v2 = v2 + this.F_par(i, 2)*cos(wf*i*t) + this.F_par(i+5, 2)*sin(wf*i*t);
                            a2 = a2 + (-this.F_par(i, 2)*sin(wf*i*t) + this.F_par(i+5, 2)*cos(wf*i*t))*wf*i;
                        end
                        p1 = p1 + this.F_par(2*N+1);
                        p2 = p2 + this.F_par(2*(2*N+1));

                        rec.P(r, :) = [p1, p2];
                        rec.V(r, :) = [v1, v2];
                        rec.A(r, :) = [a1, a2];
                        r = r + 1;
                    end
                    Output = rec; 
            end   
        end
        
        %% 順向運動學 角度推位置
        % 輸入 : 角度
        % 輸出 : 位置
        function Output = Direc_Kinematics(this, P)
           x = this.L(1).*cos(P(:, 1)) + this.L(2).*cos(P(:, 1) + P(:, 2));
           y = this.L(1).*sin(P(:, 1)) + this.L(2).*sin(P(:, 1) + P(:, 2)); 
           Output = [x, y];
        end
        
        %% 逆向運動學  位置找角度
        % 輸入 : 位置
        % 輸出 : struct P,V,A (角度,角速度,角加速度)
        function Output = Inverse_Kinematics(this, P)
            Theta2 = acos( (P(:,1).^2 + P(:,2).^2 - this.L(1)^2 - this.L(2)^2) ...
                ./  (2*this.L(1)*this.L(2)) );
            Theta1 = atan( ((this.L(1) + this.L(2).*cos(Theta2)).*P(:,2) - this.L(2).*sin(Theta2).*P(:,1))...
                ./   ((this.L(1) + this.L(2).*cos(Theta2)).*P(:,1) + this.L(2).*sin(Theta2).*P(:,2)) );
             
            dTheta = derivate([Theta1, Theta2], 1, this.sampT, this.axis);
            ddTheta = derivate([Theta1, Theta2], 2, this.sampT, this.axis);
            
            rec.P = [Theta1, Theta2];
            rec.V = dTheta;
            rec.A = ddTheta;
            Output = rec;
        end
        
        %% 模擬
        % 輸入 : 角度,角速度,角加速度命令
        % 輸出 : struct P,V,A (角度,角速度,角加速度)
        function [Output, Stable] = Simulation(this, Command)
            P = [Command.P(1, 1); Command.P(1, 2)];
            [V, A] = deal(zeros(this.axis,1));
            
            PCmd = [Command.P(:, 1)'; Command.P(:, 2)'];
            VCmd = [Command.V(:, 1)'; Command.V(:, 2)'];
            ACmd = [Command.A(:, 1)'; Command.A(:, 2)'];
            T = this.sampT : this.sampT : length(Command.P(:, 1))*this.sampT;
            nc = length(T);
            
            [rec.P, rec.V, rec.A, rec.T, rec.F] = deal(zeros(nc ,this.axis));
             
            i = 2; Stable = true;
            while (i < nc+2)
                % M, C
                switch this.controller.type
                    case 'Linear'
                        M = this.M_Full(PCmd);
                        C = this.C_Full(PCmd, VCmd);
                    case 'CTC'
                        M = this.M_Full(PCmd);
                        C = this.C_Full(PCmd, VCmd);
                    case 'DFF'
                        M = this.M_Full(P);
                        C = this.C_Full(P, V);
                end
                
                ControllerTorque = this.controller.Torque(PCmd(:, i-1), P, ...
                                                     VCmd(:, i-1), V, ...
                                                     ACmd(:, i-1), M, C);      
                % Calculate Friction
                F = 0;
                
                % Calculate A || DDM Diret Dynamic Model
                A = M \ (ControllerTorque - C * V );
                
                % Integral V, P
                V = V + A * this.sampT;
                P = P + V * this.sampT;
                
                % record
                rec.P(i-1, :) = P';
                rec.V(i-1, :) = V';
                rec.A(i-1, :) = A';
                rec.T(i-1, :) = ControllerTorque';
                rec.F(i-1, :) = F';
                
                if sum(isinf(A)) > 0
                    Stable = false;
                    break
                end
                i = i + 1;  
            end
            Output = rec;
        end

        %% 慣量矩陣
        % 輸入 : 角度
        % 輸出 : 慣量矩陣
        function   Output = M_Full(this, P)      
            Output =...
                [this.I(1) + this.I(2) + this.m(1)*this.c(1)^2 + this.m(2)*(this.L(1)^2 + ...
                          this.c(2)^2 + 2*this.L(1)*this.c(2)*cos(P(2))), ...
                 this.I(2) + this.m(2)*this.c(2)^2 + this.m(2)*this.L(1)*this.c(2)*cos(P(2));
                 this.I(2) + this.m(2)*this.c(2)^2 + this.m(2)*this.L(1)*this.c(2)*cos(P(2)), ...
                 this.I(2) + this.m(2)*this.c(2)^2];
        end
        
        %% 科氏力矩陣
        % 輸入 : 1.角度 2.角速度
        % 輸出 : 科氏力矩陣
        function   Output = C_Full(this, P, V)      
            Output =...
                [-1*this.m(2)*this.L(1)*this.c(2)*V(2,1)*sin(P(2,1)), ...
                 -1*this.m(2)*this.L(1)*this.c(2)*(V(1,1) + V(2,1))*sin(P(2,1));
                 this.m(2)*this.L(1)*this.c(2)*V(1,1)*sin(P(2,1)) ...
                 , 0];
        end
        
        %% 位置誤差
        % 輸入 : 控制器類別
        % 輸出 : 位置誤差
        function Output = PosError(this)
            Command = this.Trajectory();
            [sim, Stable] = this.Simulation(Command);
            sim = sim.P(:, 1 : this.axis);
            P_sim = this.Direc_Kinematics(sim);
            P_des = this.Direc_Kinematics(Command.P(:, 1 : this.axis));
            
            if Stable
                Output = sqrt((P_sim(:, 1) - P_des(:, 1)) .^2 + (P_sim(:, 2) - P_des(:, 2)) .^2);
            else
                Output = Inf;
            end
            
        end
        
       %% 慣量矩陣 scara 
       function Output = H_Full(this, P)
           H = [this.Theta(1) + this.Theta(2) + 2*this.L(1)*(this.Theta(3)*cos(P(2)) - this.Theta(4)*sin(P(2))), ...
                this.Theta(2) + this.Theta(3)*this.L(1)*cos(P(2)) - this.Theta(4)*this.L(1)*sin(P(2)); ...
                this.Theta(2) + this.Theta(3)*this.L(1)*cos(P(2)) - this.Theta(4)*this.L(1)*sin(P(2)), ...
                this.Theta(5) + this.Theta(2)];
            Output = H;
       end
       
       %% 剩餘力矩陣 scara 
       function Output = GAM_Full(this, P, V)
           sigma1 = this.Theta(3)*this.L(1)*V(1)^2*sin(P(2));
           sigma2 = this.Theta(4)*this.L(1)*V(1)^2*cos(P(2));
           Output = [this.Theta(8)*V(1) + this.Theta(6)*sign(V(1)) - this.L(1)*((this.Theta(4)*cos(P(2)) + this.Theta(3)*sin(P(2)))*(V(1) + V(2))^2) + sigma1 + sigma2; ...
                     this.Theta(9)*V(2) + this.Theta(7)*sign(V(2)) + sigma1 + sigma2];
       end

       %% 測試 鑑別參數
       function Output = par_test(this)
           Command = this.Trajectory();
           P = [Command.P(1, 1); Command.P(1, 2)];
           [V, A] = deal(zeros(this.axis,1));
            
           PCmd = [Command.P(:, 1)'; Command.P(:, 2)'];
           VCmd = [Command.V(:, 1)'; Command.V(:, 2)'];
           ACmd = [Command.A(:, 1)'; Command.A(:, 2)'];
           T = this.sampT : this.sampT : length(Command.P(:, 1))*this.sampT;
           nc = length(T);
          [rec.P, rec.V, rec.A, rec.T, rec.F] = deal(zeros(nc ,this.axis));
            
           i = 2; Stable = true;
           while (i < nc+2)
               % H, GAM
               H = this.H_Full(P);
               GAM = this.GAM_Full(P, V);
               
               ControllerTorque = this.controller.Torque(PCmd(:, i-1), P, ...
                                                    VCmd(:, i-1), V, ...
                                                    ACmd(:, i-1), H, GAM);      
               % Calculate Friction
               F = 0;
               
               % Calculate A || DDM Diret Dynamic Model
               A = H \ (ControllerTorque - GAM );                
                
               % Integral V, P
               V = V + A * this.sampT;
               P = P + V * this.sampT;
                
               % record
               rec.P(i-1, :) = P';
               rec.V(i-1, :) = V';
               rec.A(i-1, :) = A';
               rec.T(i-1, :) = ControllerTorque';
               rec.F(i-1, :) = F';
                
               if sum(isinf(A)) > 0
                   Stable = false;
                   break
               end
               i = i + 1;  
           end
           sim = rec.P(:, 1 : this.axis);
           P_sim = this.Direc_Kinematics(sim);
           des = this.Trajectory();
           P_des = this.Direc_Kinematics(des.P);
           
           if Stable
               Output = sqrt((P_sim(:, 1) - P_des(:, 1)) .^2 + (P_sim(:, 2) - P_des(:, 2)) .^2);
           else
               Output = Inf;
           end           
        end
    end
end