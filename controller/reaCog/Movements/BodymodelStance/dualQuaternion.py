'''
Created on 9.09.2011

@author: mschilling
'''
import controller.reaCog.Movements.BodymodelStance.pycgtypes.quat as quat
import controller.reaCog.Movements.BodymodelStance.pycgtypes.vec3 as vec3
import math


class dualQuaternion:
    """Dual Quaternion class
        two quaternions are used to represent transformations
        in general
            - real part: represents a rotation
            - dual part: represents a translation

        As a geometric interpretation, a dual quaternion describes
        a screw motion - i.e. a rotation around an axis and a
        translation along this axis. It is defined by the angle of
        rotation, the length of translation, the axis of rotation
        (and direction of translation) as well as the offset of this
        center of rotation (the axis).

        Transformations are concatenated by right multiplication
        (with respect to the new transformed coordinate frame).

        By convention the transformations are provided in a half
        notation = half angle of rotation, half translation because
        when actually used for calculating positions, one
        is applying the dual quaternion from both sides:
            q * v * q^-1

    """
    ##    A dual quaternion can be initialized by giving it a
    #        - dual quaternion (makes a copy of real and dual part)
    #        - two quaternions (real, dual part)
    #        - two lists with four components (one for each quaternion,
    #          first = real, second = dual part)
    #        - no argument: a null transformation is initialized
    def __init__(self, *args):
        if len(args)==0 :
            self.real_part = quat([1,0,0,0])
            self.dual_part = quat([0,0,0,0])
        elif len(args)==1:
            # One DualQuaternion is given
            if isinstance(args[0], dualQuaternion):
                self.real_part = quat(args[0].real_part)
                self.dual_part = quat(args[0].dual_part)
            # All the values are given in one list
            else:
                self.real_part = (quat((args[0])[0:3])).normalize()
                self.dual_part = quat((args[0])[4:7])
        elif len(args)==2:
            # Two quaternions are given
            if isinstance(args[0], quat):
                self.real_part = quat(args[0])
                self.dual_part = quat(args[1])
            # Two lists are given
            else :
                self.real_part = (quat(args[0])).normalize()
                self.dual_part = quat(args[1])
                
    ## Initialize by providing an Angle and an axis - as a vector (.,.,.)
    def set_rotation (self, rot_angle, rot_axis):
        self.real_part = quat(rot_angle, rot_axis)

    ## Initialize by providing a Translation as a list
    def set_translation (self, transl_list):
        self.dual_part = quat([0] + [x*0.5 for x in transl_list])

    ##    Multiplication - of two Dual quaternions or scaling.
    #    The dual quaternion is (post-)multiplied with
    #        - a scalar (float, int or long)
    #        - another dual quaternion
    def __mul__ (self,post_op):
        T = type(post_op)
        # Dual Quaternion * scalar
        if T==float or T==int or T==int:
            return dualQuaternion(self.real_part*post_op, self.dual_part*post_op)
        # DualQuaternion*DualQuaternion
        if isinstance(post_op, dualQuaternion):
            return dualQuaternion(self.real_part * post_op.real_part,
                                  self.real_part * post_op.dual_part +
                                  self.dual_part * post_op.real_part)
        else:
            raise TypeError("unsupported operand type for *")

    ##    Output method for dual quaternion:
    #    first real and then dual part is given back.
    def __str__(self):
        return '('+self.real_part.__str__()+' '+self.dual_part.__str__()+')'

    ##    Conjugation is working on each of the quaternions individually
    #    but for the translational part one also has to invoke the
    #    conjugate of the dual - therefore the -1
    def get_conjugate(self):
        return dualQuaternion(self.real_part.conjugate(),self.dual_part.conjugate()*-1)

    ##    Inverse is computed for the real part as for the quaternions
    #    (means it is switching signs and scaling by the lenght which
    #    has no effect as the rotations are unit length)
    #    Again, for the dual part this is different - the inverse is switching
    #    signs but no scaling. From the geometric point of view this becomes
    #    obvious as there is no reason to normalize an inverse translation to unit
    #    length. Therefore, we use simply conjugate here (the mathematical
    #    explanation involves the definition of the dual quaternion multiplication)
    def inverse(self):
        return dualQuaternion(self.real_part.inverse(),self.dual_part.conjugate())

    ##    Normalize the dual quaternion - afterwards the real part
    #    is unit length.
    def normalize(self):
        w_dq = abs(self.real_part)
        self.real_part = self.real_part * 1/w_dq
        # ??? Scale also dual part?
        self.dual_part = self.dual_part * 1/w_dq

    ##    For joint transformations: only rotations should be left -
    #    the translational part should be cancelled (or compensated)
    def strip_translation(self):
        self.dual_part = quat([0,0,0,0])

    ##    For segment transformations: only translations should be left -
    #    the rotational part should be cancelled (or compensated)
    def strip_rotation(self):
        self.real_part = quat([1,0,0,0])

    ##    For segment transformations: only translations should be left -
    #    the rotational part (real part) can be compensated.
    #    Here, the complete displacement is calculated - what is lost
    #    is the orientation of the endpoint
    def compensate_rotation(self):
        self.dual_part = (self * self.get_conjugate()).dual_part * 0.5
        self.real_part = quat([1,0,0,0])

    ##    For the restricted joint the resulting quaternion describing the rotation
    #    is projected onto the closest quaternion with the given and
    #    fixed axis of rotation. This is done by only considering the displacement
    #    described by the initial rotation that lies in the plane
    #    in which the joint operates (i.e. the plane perpendicular to the joint axis)
    #    and deriving from this the rotational angle.
    def project_onto_fixed_rotation_around_y(self, segm_dq):
        if (self.real_part.toAngleAxis()[0] > 0) :
            rot_angle = -math.atan2((self.real_part * segm_dq.dual_part * self.real_part.conjugate()).z, \
                         (self.real_part * segm_dq.dual_part * self.real_part.conjugate()).x )
            self.real_part = quat(rot_angle, vec3(0,1,0))

    ##    For the restricted joint the resulting quaternion describing the rotation
    #    is projected onto the closest quaternion with the given and
    #    fixed axis of rotation. This is done by only considering the displacement
    #    described by the initial rotation that lies in the plane
    #    in which the joint operates (i.e. the plane perpendicular to the joint axis)
    #    and deriving from this the rotational angle.
    def project_onto_fixed_rotation_around_z(self, segm_dq):
        if (self.real_part.toAngleAxis()[0] > 0) :
            rot_angle = math.atan2((self.real_part * segm_dq.dual_part * self.real_part.conjugate()).y, \
                         (self.real_part * segm_dq.dual_part * self.real_part.conjugate()).x )
            self.real_part = quat(rot_angle, vec3(0,0,1))

    ##    For joint transformations: only rotations should be left -
    #    the translational part (dual part) can be partly compensated.
    #    The part of the displacement which is orthogonal to the outgoing
    #    segment can be compensated through rotation of the joint.
    #    The part along the following segment must be simply dropped
    def compensate_translation(self, along_seg, scale=1):
        # Here the translational error is calculated.
        # ! With respect to the rotated reference frame!
        # Therefore here the order has to be quat.conjugate * quat
        # - not the other way around
        error_dq = self.get_conjugate() * self
        # Because the multiplication is doubling the length,
        # it has to be multiplied by 0.5 and can be scaled
        error_dq.dual_part = error_dq.dual_part * scale * 0.5
        # The two translational parts of the dual quaternions are
        # transformed to vectors - for using dot product
        #error_vec = vec3(error_dq.dual_part.x,error_dq.dual_part.y,error_dq.dual_part.z)
        #along_vec = vec3(along_seg.dual_part.x,along_seg.dual_part.y,along_seg.dual_part.z)
        dist = along_seg.dual_part.dot(error_dq.dual_part)/abs(along_seg.dual_part)
        # The subtracted term is the projection of the error term onto
        # the other vector (the translation along the segment)
        # The result is therefore the share of the error which is
        # orthogonal.
        error_dq.dual_part = error_dq.dual_part - dist * along_seg.dual_part.normalize()
        # Only the share of the error which is orthogonal to the
        # outgoing segment shall be used.
        # First: the translational share is dropped.
        self.strip_translation()
        # Then the current rotation is furthermore rotated by a rotation which
        # would align the current outgoing segment with the direction of
        # the intended orientation (given through the concatenation of the
        # segment and the orthogonal error).
        self.real_part=self.real_part*(along_seg.get_aligning_rotation(along_seg*error_dq))

    ##    Returns a rotation (a quaternion) which rotates self (translational
    #    part) onto the target dual quaternion (translational part)
    def get_aligning_rotation(self, target):
        start_vec = vec3(self.dual_part.x,self.dual_part.y,self.dual_part.z)
        target_vec = vec3(target.dual_part.x,target.dual_part.y,target.dual_part.z)
        try:
            comp_angle = start_vec.angle(target_vec)
        except ValueError:
            comp_angle = 0.0
            #print "ValueError caught"
        comp_axis = start_vec.cross(target_vec)
        return quat([comp_angle,comp_axis])

    ##    Linear interpolation between a list of dual quaternions.
    #    In addition, a list with weights has to be provided:
    #    each dual quaternion is weighted by the corresponding factor
    #    of this second list.
    #    Linear interpolation simply calculates a mean dual quaternion
    #    and then normalizes (with respect to the real part) it.
    def linear_blending(self,quat_list,weights):
        sum_weights=sum(weights)
        result_quat_real = quat([0,0,0,0])
        result_quat_dual = quat([0,0,0,0])
        # Add up the dual quaternions.
        for i in range(len(quat_list)):
            result_quat_real = result_quat_real + \
                quat_list[i].real_part * weights[i]/sum_weights
            result_quat_dual = result_quat_dual + \
                quat_list[i].dual_part * weights[i]/sum_weights
        self.real_part = result_quat_real
        self.dual_part = result_quat_dual
        self.normalize()
