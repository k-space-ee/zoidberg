import numpy as np
import cv2
import os
from managed_threading import ManagedThread

class Visualizer(ManagedThread):

    def step(self, r, grabber):

        frame = np.swapaxes(cv2.cvtColor(r.frame.reshape((-1, 640, 2)), cv2.COLOR_YUV2BGR_YUYV), 0, 1).copy()

        """
#        frame = frame[2160-1500:2160+1500]


        if r.goal_yellow:
            x = r.deg_to_x(r.goal_yellow.angle_deg)
            cv2.line(frame, (x,0), (x,r.GOAL_BOTTOM-120), (255,255,255), 3)
            cv2.putText(frame, "%.1fdeg" % r.goal_yellow.angle_deg, (x, r.GOAL_BOTTOM-80), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,0), 4)
            cv2.putText(frame, "%.1fm" % r.goal_yellow.dist, (x, r.GOAL_BOTTOM-30), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,0), 4)
            for rect in r.goal_yellow_rect:
                cv2.rectangle(frame, (rect[0], rect[1]), (rect[2]+rect[0], rect[3]+rect[1]), (255,255,255), 4)

        if r.goal_blue:
            x = r.deg_to_x(r.goal_blue.angle_deg)
            cv2.line(frame, (x,0), (x,r.GOAL_BOTTOM-120), (255,255,255), 3)
            cv2.putText(frame, "%.1fdeg" % r.goal_blue.angle_deg, (x, r.GOAL_BOTTOM-80), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,0), 4)
            cv2.putText(frame, "%.1fm" % r.goal_blue.dist, (x, r.GOAL_BOTTOM-30), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,0), 4)
            for rect in r.goal_blue_rect:
                cv2.rectangle(frame, (rect[0], rect[1]), (rect[2]+rect[0], rect[3]+rect[1]), (255,255,255), 4)

        index = 0
        for relative, absolute, x, y, radius in r.balls:

            cv2.circle(frame, (x,y), radius, (255,255,255) if index else (0,0,255), 3)
            if index == 0:
                x = r.deg_to_x(relative.angle_deg)
                cv2.putText(frame, "%.1fdeg" % relative.angle_deg, (x + 20, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,192,255), 4)
                cv2.putText(frame, "%.1fm" % relative.dist, (x + 20, y + 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,192,255), 4)

            index += 1



        scope = 1500
#        frame = frame[:,frame.shape[1]-scope:frame.shape[1]+scope]
        frame = frame[:,660:3660]

#        frame = np.vstack([frame[:,-240+1080:1080+2160+240],
#            np.hstack([frame[:,240:1080], frame[:,1080+2160:-240]])])


        ret, jpeg = cv2.imencode('.jpg', frame, (cv2.IMWRITE_JPEG_QUALITY, 50))
        buf = jpeg.tostring()
        self.produce(buf)

        return
        """

        field_mask = np.swapaxes(np.repeat(r.field_mask, 2, axis=1), 0, 1)
        goal_blue_mask = np.swapaxes(np.repeat(r.goal_blue_mask, 2, axis=1), 0, 1)

        field_cutout = cv2.bitwise_and(frame, frame, mask=field_mask)
        cv2.line(field_cutout, (0,0), (5000,0), (255,255,255), 2)
        cv2.putText(field_cutout, "field detection", (80, field_cutout.shape[0] >> 1), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)




        # Visualize yellow goal
        sliced = frame[:r.BALLS_BOTTOM*2-r.GOAL_FIELD_DILATION*2,:]
        goal_yellow_mask = np.swapaxes(np.repeat(r.goal_yellow_mask, 2, axis=1), 0, 1)
        goal_yellow_cutout = cv2.bitwise_and(sliced, sliced, mask=goal_yellow_mask)
        cv2.line(goal_yellow_cutout, (0,0), (5000,0), (255,255,255), 2)
        cv2.putText(goal_yellow_cutout, "yellow goal detection", (80, goal_yellow_cutout.shape[0] >> 1), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)

        if r.goal_yellow:
            x = r.deg_to_x(r.goal_yellow.angle_deg)
            cv2.line(goal_yellow_cutout, (x,0), (x,r.GOAL_BOTTOM-120), (255,255,255), 3)
            cv2.putText(goal_yellow_cutout, "%.1fdeg" % r.goal_yellow.angle_deg, (x, r.GOAL_BOTTOM-80), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,0), 4)
            cv2.putText(goal_yellow_cutout, "%.1fm" % r.goal_yellow.dist, (x, r.GOAL_BOTTOM-30), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,0), 4)
            for rect in r.goal_yellow_rect:
                cv2.rectangle(goal_yellow_cutout, (rect[0], rect[1]), (rect[2]+rect[0], rect[3]+rect[1]), (255,255,255), 4)

        # Visualize blue goal
        sliced = frame[:r.BALLS_BOTTOM*2-r.GOAL_FIELD_DILATION*2,:]
        goal_blue_mask = np.swapaxes(np.repeat(r.goal_blue_mask, 2, axis=1), 0, 1)
        goal_blue_cutout = cv2.bitwise_and(sliced, sliced, mask=goal_blue_mask)
        cv2.line(goal_blue_cutout, (0,0), (5000,0), (255,255,255), 2)
        cv2.putText(goal_blue_cutout, "blue goal detection", (80, goal_blue_cutout.shape[0] >> 1), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)

        if r.goal_blue:
            x = r.deg_to_x(r.goal_blue.angle_deg)
            cv2.line(goal_blue_cutout, (x,0), (x,r.GOAL_BOTTOM-120), (255,255,255), 3)
            cv2.putText(goal_blue_cutout, "%.1fdeg" % r.goal_blue.angle_deg, (x, r.GOAL_BOTTOM-80), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,0), 4)
            cv2.putText(goal_blue_cutout, "%.1fm" % r.goal_blue.dist, (x, r.GOAL_BOTTOM-30), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,0), 4)
            for rect in r.goal_blue_rect:
                cv2.rectangle(goal_blue_cutout, (rect[0], rect[1]), (rect[2]+rect[0], rect[3]+rect[1]), (255,255,255), 4)

        sliced = frame[:r.BALLS_BOTTOM*2]
        balls_mask = np.swapaxes(np.repeat(r.balls_mask, 2, axis=1), 0, 1)
        balls_cutout = cv2.bitwise_and(sliced, sliced, mask=balls_mask)
        cv2.line(balls_cutout, (0,0), (5000,0), (255,255,255), 2)
        cv2.putText(balls_cutout, "balls detection", (80, balls_cutout.shape[0] >> 1), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
        index = 0
        for relative, absolute, x, y, radius in r.balls:

            cv2.circle(frame, (x,y), radius, (255,255,255) if index else (0,0,255), 3)
            cv2.putText(frame, "%.1fdeg" % relative.angle_deg, (x+radius+10, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,0), 4)
            cv2.putText(frame, "%.1fm" % relative.dist, (x+radius+10, y+30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,0), 4)
            cv2.putText(frame, "%dpx" % radius, (x+radius+10, y+50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,0), 4)

            index += 1


        def left_top(args):
            y,x  = args
            return 4320-x-4, y*2-4

        def right_bottom(args):
            y,x = args
            return 4320-x+4, y*2+4

        cv2.putText(balls_cutout, "Got it!" if r.ball_grabbed else "Lost!", (2212, 500), cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),1)

        cv2.rectangle(balls_cutout, left_top(r.ball_grabbed_orange), right_bottom(r.ball_grabbed_orange), (255,255,255), 2) #[588][2212][0]
        cv2.rectangle(balls_cutout, left_top(r.ball_grabbed_green1), right_bottom(r.ball_grabbed_green1), (255,255,255), 2) #[588][2212][0]
        cv2.rectangle(balls_cutout, left_top(r.ball_grabbed_green2), right_bottom(r.ball_grabbed_green2), (255,255,255), 2) #[588][2212][0]



        frame = np.vstack([
            frame,
            field_cutout,
            goal_blue_cutout,
            goal_yellow_cutout,
            balls_cutout,
        ])

        cv2.rectangle(frame, left_top(r.ball_grabbed_orange), right_bottom(r.ball_grabbed_orange), (255,255,255), 2) #[588][2212][0]
        cv2.rectangle(frame, left_top(r.ball_grabbed_green1), right_bottom(r.ball_grabbed_green1), (255,255,255), 2) #[588][2212][0]
        cv2.rectangle(frame, left_top(r.ball_grabbed_green2), right_bottom(r.ball_grabbed_green2), (255,255,255), 2) #[588][2212][0]

        ret, jpeg = cv2.imencode('.jpg', frame, (cv2.IMWRITE_JPEG_QUALITY, 50))
        buf = jpeg.tostring()
        self.produce(buf)

