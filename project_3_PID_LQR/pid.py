class PIDController:
    def __init__(self, target_pos):
        self.target_pos = target_pos
        self.Kp = 4000
        self.Ki = 1000
        self.Kd = 3000
        self.bias = 0.0
        self.p_term = 0
        self.i_term = 0
        self.d_term = 0
        self.perror = 0
        self.windup_guard = 20
        return

    #Global value Iterm 
    #i_term = 0
    #Error at the previous time step initial:0
    

    def reset(self):
        self.Kp = 4000
        self.Ki = 1000
        self.Kd = 3000
        self.bias = 0.0
        self.p_term = 0
        self.i_term = 0
        self.d_term = 0
        self.perror = 0
        self.windup_guard = 20
        return

#TODO: Complete your PID control within this function. At the moment, it holds
#      only the bias. Your final solution must use the error between the 
#      target_pos and the ball position, plus the PID gains. You cannot
#      use the bias in your final answer. 
    def get_fan_rpm(self, vertical_ball_position):

        error = self.target_pos - vertical_ball_position
        d_error = error - self.perror
        self.perror = error
        #d_error is delta error 

        # delta time is 1/60 said in the discussion 
        d_time =  0.0166

        # p
        self.p_term = error
        # i
        self.i_term += error * d_time

        #windup
        if (self.i_term < -self.windup_guard):
                self.i_term = -self.windup_guard
        elif (self.i_term > self.windup_guard):
                self.i_term = self.windup_guard
        # d
        if (d_time > 0):
            self.d_term = d_error / d_time

        output = (self.Kp * self.p_term) + (self.Ki * self.i_term) + (self.Kd * self.d_term)
        return output
