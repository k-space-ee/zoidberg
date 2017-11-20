import numpy as np
import cv2

from line_fit import dist_to_pwm
from managed_threading import ManagedThread


def left_top(args):
    y,x  = args
    return x-4, y*2-4

def right_bottom(args):
    y,x = args
    return x+4, y*2+4

class Visualizer(ManagedThread):
    ZOOM = 0.5 # 0.2
    DEBUG_MASK = False
    def step(self, r, grabber):
        frame = r.frame
        if frame.shape[0] == 3840:
            frame = np.vstack([frame, frame[:480]])
        converted = np.swapaxes(cv2.cvtColor(frame.reshape((4320, 640, 2)), cv2.COLOR_YUV2BGR_YUYV), 0, 1)
        frame = converted.copy() # This speeds up whole lot

        # Kicker offset
        cv2.line(frame, (r.kicker_offset, frame.shape[0]-50), (r.kicker_offset, frame.shape[0]), (255,255,255), 3)

        # Visualize closest edge
        closest_x, closest_y = r.deg_to_x(r.closest_edge.angle_deg), r.dist_to_y(r.closest_edge.dist)
        cv2.putText(frame, "edge %.2fm" % r.closest_edge.dist, (closest_x-240, 600), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,0), 4)

        # Visualize center of field
        center_x = r.deg_to_x(r.field_center.angle_deg)
        cv2.line(frame, (center_x,0), (center_x,480), (255,255,255), 3)
        cv2.putText(frame, "cnt %.1fdeg %.2fm" % (r.field_center.angle_deg, r.field_center.dist), (center_x, 450), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,0), 4)

        if r.goal_yellow:
            dist = r.goal_yellow.dist * 100
            pwm = dist_to_pwm(dist)
            cv2.putText(frame, "DIST %.0f" % (dist), (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,0), 4)
            cv2.putText(frame, "PWM %.0f" % (pwm), (100, 150), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,0), 4)

        # Visualize field edges
        points = []
        for index, hull in enumerate(r.field_contours):
            y,x,h,w = cv2.boundingRect(hull)
            rect = (x+index*480),2*y,w,h*2
            points.append((int(x+index*480+w/2), 2*y))
        if points:
            points = [(points[-1][0] - 3840, points[-1][1])] + points
            prev = None
            for point in points:
                if prev is not None:
                    cv2.line(frame, prev, point, (128,255,128),4)
                prev = point

        # Visualize balls
        index = 0
        prev = (r.kicker_offset, 640)
        for relative, absolute, x, y, radius in r.balls:
            cv2.circle(frame, (x,y), radius, (255,255,255) if index else (0,0,255), 3)
            x = r.deg_to_x(relative.angle_deg)
            cv2.putText(frame, "%.1fdeg" % relative.angle_deg, (x + 20, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,192,255), 4)
            cv2.putText(frame, "%.2fm" % relative.dist, (x + 20, y + 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,192,255), 4)
            index += 1
            if index < 5:
                point = x, y
                if prev is not None:
                    cv2.line(frame, prev, point, (0,0,255),4)
                prev = point


        # Visualize goals
        if r.goal_yellow:
            for delta in -3840, 0, 3840:
                x = r.deg_to_x(r.goal_yellow.angle_deg) + delta
                cv2.line(frame, (x,0), (x,r.GOAL_BOTTOM-120), (255,255,255), 3)
                cv2.putText(frame, "%.1fdeg" % r.goal_yellow.angle_deg, (x, r.GOAL_BOTTOM-80), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,0), 4)
                cv2.putText(frame, "%.2fm" % r.goal_yellow.dist, (x, r.GOAL_BOTTOM-30), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,0), 4)
                cv2.putText(frame, "%.1fdeg wide" % r.goal_yellow_width_deg, (x, r.GOAL_BOTTOM+20), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,0), 4)
            for rect in r.goal_yellow_rect:
                cv2.rectangle(frame, (rect[0], rect[1]), (rect[2]+rect[0], rect[3]+rect[1]), (255,255,255), 4)

        if r.goal_blue:
            for delta in -3840, 0, 3840:
                x = r.deg_to_x(r.goal_blue.angle_deg) + delta
                cv2.line(frame, (x,0), (x,r.GOAL_BOTTOM-120), (255,255,255), 3)
                cv2.putText(frame, "%.1fdeg" % r.goal_blue.angle_deg, (x, r.GOAL_BOTTOM-80), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,0), 4)
                cv2.putText(frame, "%.2fm" % r.goal_blue.dist, (x, r.GOAL_BOTTOM-30), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,0), 4)
                cv2.putText(frame, "%.1fdeg wide" % r.goal_blue_width_deg, (x, r.GOAL_BOTTOM+20), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,0), 4)
            for rect in r.goal_blue_rect:
                cv2.rectangle(frame, (rect[0], rect[1]), (rect[2]+rect[0], rect[3]+rect[1]), (255,255,255), 4)


        if not self.DEBUG_MASK: # TODO: Read from config manager
            resized = cv2.resize(frame, (0,0), fx=self.ZOOM, fy=self.ZOOM)
            ret, jpeg = cv2.imencode('.jpg', frame, (cv2.IMWRITE_JPEG_QUALITY, 50))
            buf = jpeg.tostring()
            resized = frame
            self.produce(buf, resized, frame, r)
            return


        # Visualize field mask
        field_mask = np.swapaxes(np.repeat(r.field_mask, 2, axis=1), 0, 1)
        field_mask = np.hstack([field_mask, field_mask[:,:480]])
        #print("DEBUG Visualize %s"%str(type(field_mask)))
        #print("DEBUG Visualize %s"%str((field_mask.shape)))

        print("converted:", converted.shape, "mask:", field_mask.shape)
        field_cutout = cv2.bitwise_and(converted, converted, mask=field_mask)
        cv2.line(field_cutout, (0,0), (5000,0), (255,255,255), 2)
        cv2.putText(field_cutout, "field detection", (80, field_cutout.shape[0] >> 1), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)


        # Visualize yellow mask
        sliced = converted[:r.BALLS_BOTTOM*2-r.GOAL_FIELD_DILATION*2,:]
        goal_yellow_mask = np.swapaxes(np.repeat(r.goal_yellow_mask, 2, axis=1), 0, 1)
        goal_yellow_mask = np.hstack([goal_yellow_mask, goal_yellow_mask[:,:480]])
        goal_yellow_cutout = cv2.bitwise_and(sliced, sliced, mask=goal_yellow_mask)
        cv2.line(goal_yellow_cutout, (0,0), (5000,0), (255,255,255), 2)
        cv2.putText(goal_yellow_cutout, "yellow goal detection", (80, goal_yellow_cutout.shape[0] >> 1), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)


        # Visualize blue mask
        sliced = converted[:r.BALLS_BOTTOM*2-r.GOAL_FIELD_DILATION*2,:]
        goal_blue_mask = np.swapaxes(np.repeat(r.goal_blue_mask, 2, axis=1), 0, 1)
        goal_blue_mask = np.hstack([goal_blue_mask, goal_blue_mask[:,:480]])
        goal_blue_cutout = cv2.bitwise_and(sliced, sliced, mask=goal_blue_mask)
        cv2.line(goal_blue_cutout, (0,0), (5000,0), (255,255,255), 2)
        cv2.putText(goal_blue_cutout, "blue goal detection", (80, goal_blue_cutout.shape[0] >> 1), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)


        # Visualize orange balls
        sliced = converted[:r.BALLS_BOTTOM*2]
        balls_mask = np.swapaxes(np.repeat(r.balls_mask, 2, axis=1), 0, 1)
        #balls_mask = np.hstack([balls_mask, balls_mask[:,:480]])
        balls_cutout = cv2.bitwise_and(sliced, sliced, mask=balls_mask)
        cv2.line(balls_cutout, (0,0), (5000,0), (255,255,255), 2)
        cv2.putText(balls_cutout, "balls detection", (80, balls_cutout.shape[0] >> 1), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)


        prev = None
        for relative, absolute, x, y, radius in r.balls:
            cv2.circle(balls_cutout, (x,y), radius, (255,255,255) if index else (0,0,255), 3)

        frame = np.vstack([
            frame,
            field_cutout,
            goal_blue_cutout,
            goal_yellow_cutout,
            balls_cutout,
        ])


        resized = cv2.resize(frame, (0,0), fx=self.ZOOM, fy=self.ZOOM)
        ret, jpeg = cv2.imencode('.jpg', resized, (cv2.IMWRITE_JPEG_QUALITY, 80)) # 60
        buf = jpeg.tostring()
        self.produce(buf, resized, frame, r)

