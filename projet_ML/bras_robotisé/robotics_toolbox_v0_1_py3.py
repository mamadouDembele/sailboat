# coding: utf-8
# for converting pydoc output to pdf , utf8 is not recongnized by htmldoc and libreoffice is used ... 
"""
Modified version of the original toolbox from Peter Corke (BZ 2015)

Link object.
Primitive operations for 3x3 orthonormal and 4x4 homogeneous matrices.
Quaternion class.
Robot object.
Utility,Transform,Jacobian
Robot kinematic operations.

@author: Peter Corke
@copyright: Peter Corke
"""

import numpy as np
import copy

#from numpy.linalg import norm
#from numpy.linalg import pinv


class Robot(object):
    """Robot object.
    Instances of this class represent a robot manipulator
    within the toolbox.
    """
        
    def __init__(self, arg=None, gravity=None, base=None, tool=None, name='', comment='', manuf=''):
        """
        Robot object constructor.  Create a robot from a sequence of Link objects.
        
        Several basic forms exist:
            - Robot()        create a null robot
            - Robot(robot)   create a clone of the robot object
            - Robot(links)   create a robot based on the passed links
            
        Various options can be set using named arguments:
        
            - gravity; gravitational acceleration (default=[0,0,9.81])
            - base; base transform (default 0)
            - tool; tool transform (default 0)
            - name
            - comment
            - manuf
        """

        if isinstance(arg, Robot):
            for k,v in arg.__dict__.items():
                if k == "links":
                    self.__dict__[k] = copy.copy(v);           
                else:
                    self.__dict__[k] = v;           
        elif len(arg) > 1 and isinstance(arg[0], Link):
            self.links = arg;
        else:
            raise AttributeError;

        # fill in default base and gravity direction
        if gravity != None:
            self.gravity = gravity;
        else:
            self.gravity = [0, 0, 9.81];
        
        if base != None:
            self.base = base;
        else:
            self.base = np.mat(np.eye(4,4));
        
        if tool != None:
            self.tool = tool;
        else:
            self.tool = np.mat(np.eye(4,4));

        self.manuf=""
        if manuf:
            self.manuf = manuf
        self.comment=""
        if comment:
            self.comment = comment
        self.name=""
        if name:
            self.name = name

        #self.handle = [];
        #self.q = [];
        #self.plotopt = {};
        #self.lineopt = {'Color', 'black', 'Linewidth', 4};
        #self.shadowopt = {'Color', 'black', 'Linewidth', 1};

        return None;

    def __str__(self):
        s = 'ROBOT(%s, %s)' % (self.name, self.config());
        return s;
        
    def __repr__(self):
        s = '';
        if self.name:
            s += 'name: %s\n' % (self.name)
        if self.manuf:
            s += 'manufacturer: %s\n' % (self.manuf)
        if self.comment:
            s += 'commment: %s\n' % (self.comment)
        
        for link in self.links:
            s += str(link) + '\n';
        return s;   

    def __mul__(self, r2):
        r = Robot(self);        # clone the robot
        print (r)
        r.links += r2.links;
        return r;

    def copy(self):
        """
        Return a copy of the Robot object
        """
        return copy.copy(self);
               
    def ismdh(self):
        return self.mdh;
        
    def config(self):
        """
        Return a configuration string, one character per joint, which is
        either R for a revolute joint or P for a prismatic joint.
        For the Puma560 the string is 'RRRRRR', for the Stanford arm it is 'RRPRRR'.
        """
        s = '';
        
        for link in self.links:
            if link.sigma == 0:
                s += 'R';
            else:
                s += 'P';
        return s;

    def nofriction(self, all=False):
        """
        Return a Robot object where all friction parameters are zero.
        Useful to speed up the performance of forward dynamics calculations.
        
        @type all: boolean
        @param all: if True then also zero viscous friction
        @see: L{Link.nofriction}
        """
        r = Robot(self);
        r.name += "-nf";
        newlinks = [];
        for oldlink in self.links:
            newlinks.append( oldlink.nofriction(all) )
        r.links = newlinks
        return r;
        
    def showlinks(self):
        """
        Shows details of all link parameters for this robot object, including
        inertial parameters.
        """

        count = 1;
        if self.name:
            print ('name: %s'%(self.name))
        if self.manuf:
            print ('manufacturer: %s'%(self.manuf))
        if self.comment:
            print ('commment: %s'%(self.comment))
        for l in self.links:
            print ('Link %d------------------------' % count)
            l.display()
            count += 1

    def __setattr__(self, name, value):
        """
        Set attributes of the robot object
        
            - robot.name = string (name of this robot)
            - robot.comment = string (user comment)
            - robot.manuf = string (who built it)
            - robot.tool = 4x4 homogeneous tranform
            - robot.base = 4x4 homogeneous tranform
            - robot.gravity = 3-vector  (gx,gy,gz)
        """
        
        if name in ["manuf", "name", "comment"]:
            if not isinstance(value, str):
                raise ValueError('must be a string')
            self.__dict__[name] = value;
            
        elif name == "links":
            if not isinstance(value[0], Link):
                raise ValueError('not a Link object')
            self.__dict__[name] = value;
            self.__dict__['n'] = len(value);
            # set the robot object mdh status flag
            for link in self.links:
                if link.convention != self.links[0].convention:
                    raise ValueError('robot has mixed D&H link conventions')
            self.__dict__['mdh'] = self.links[0].convention == Link.LINK_MDH;
            
        elif name == "tool":
            if not ishomog(value):
                raise ValueError('tool must be a homogeneous transform')
            self.__dict__[name] = value;

        elif name == "gravity":
            v = arg2array(value);
            if len(v) != 3:
                raise ValueError('gravity must be a 3-vector')
            self.__dict__[name] = np.mat(v).T
            
        elif name == "base":
            if not ishomog(value):
                raise ValueError('base must be a homogeneous transform')
            self.__dict__[name] = value;
            
        else:
            raise AttributeError;



class Link:
    """
    LINK create a new LINK object

    A LINK object holds all information related to a robot link such as
    kinematics of the joint
        - alpha; the link twist angle
        - an; the link length
        - theta; the link rotation angle
        - dn; the link offset
        - sigma; 0 for a revolute joint, non-zero for prismatic
        
    rigid-body inertial parameters
        - I; 3x3 inertia matrix about link COG
        - m; link mass
        - r; link COG wrt link coordinate frame 3x1

    motor and transmission parameters
        - B; link viscous friction (motor referred)
        - Tc; link Coulomb friction 1 element if symmetric, else 2
        - G; gear ratio
        - Jm; inertia (motor referred)

    and miscellaneous
        - qlim; joint limit matrix [lower upper] 2 x 1
        - offset; joint coordinate offset
    Handling the different kinematic conventions is now hidden within the LINK
L    object.

    Conceivably all sorts of stuff could live in the LINK object such as
    graphical models of links and so on.

    @see: L{Robot}
    """
    
    LINK_DH = 1
    LINK_MDH = 2

    def __init__(self, alpha=0, A=0, theta=0, D=0, sigma=0, convention=LINK_DH, offset=0):
        """
        L = LINK([alpha A theta D])
        L =LINK([alpha A theta D sigma])
        L =LINK([alpha A theta D sigma offset])
        L =LINK([alpha A theta D], CONVENTION)
        L =LINK([alpha A theta D sigma], CONVENTION)
        L =LINK([alpha A theta D sigma offset], CONVENTION)

        If sigma or offset are not provided they default to zero.  Offset is a
        constant amount added to the joint angle variable before forward kinematics
        and is useful if you want the robot to adopt a 'sensible' pose for zero
        joint angle configuration.

        The optional CONVENTION argument is 'standard' for standard D&H parameters 
        or 'modified' for modified D&H parameters.  If not specified the default
        'standard'.
        """
        self.alpha = alpha
        self.A = A
        self.theta = theta
        self.D = D
        self.sigma = sigma
        self.convention = convention
        self.offset = offset

        # we know nothing about the dynamics
        self.m = None
        self.r = None
        self.v = None
        self.I = None
        self.Jm = None
        self.G = None
        self.B = None
        self.Tc = None
        self.qlim = None

        return None

    def __repr__(self):

        if self.convention == Link.LINK_DH:
            conv = 'std'
        else:
            conv = 'mod'

        if self.sigma == 0:
            jtype = 'R'
        else:
            jtype = 'P'

        if self.D == None:
            return "alpha=%f, A=%f, theta=%f, offset=%f jtype: (%c) conv: (%s)" % (self.alpha,
                 self.A, self.theta, self.offset, jtype, conv)
        elif self.theta == None:
            return "alpha=%f, A=%f, D=%f, offset=%f jtype: (%c) conv: (%s)" % (self.alpha,
                 self.A, self.D, self.offset, jtype, conv)
        else:
            return "alpha=%f, A=%f, theta=%f, D=%f, offset=%f jtype: (%c) conv: (%s)" % (self.alpha,
                 self.A, self.theta, self.D, self.offset, jtype, conv)

    # invoked at print
    def __str__(self):
        if self.convention == Link.LINK_DH:
            conv = 'std'
        else:
            conv = 'mod'

        if self.sigma == 0:
            jtype = 'R'
        else:
            jtype = 'P'

        if self.D == None:
            return "alpha = %f\tA = %f\ttheta = %f\t--\toffset = %f\tjtype: %c\tconv: (%s)" % (
                self.alpha, self.A, self.theta, self.offset, jtype, conv)
        elif self.theta == None:
            return "alpha = %f\tA = %f\t--\tD = %f\t--\toffset = %f\tjtype: %c\tconv: (%s)" % (
                self.alpha, self.A, self.D, self.offset, jtype, conv)
        else:
            return "alpha = %f\tA = %f\ttheta = %f\tD=%f\toffset = %f\tjtype: %c\tconv: (%s)" % (
                self.alpha, self.A, self.theta, self.D, self.offset, jtype, conv)


    def display(self):

        print (self);

        if self.m != None:
            print ("m:", self.m)
        if self.r != None:
            print ("r:", self.r)
        if self.I != None:
            print ("I:\n", self.I)
        if self.Jm != None:
            print ("Jm:", self.Jm)
        if self.B != None:
            print ("B:", self.B)
        if self.Tc != None:
            print ("Tc:", self.Tc)
        if self.G != None:
            print ("G:", self.G)
        if self.qlim != None:
            print ("qlim:\n", self.qlim)

    def copy(self):
        """
        Return copy of this Link
        """
        return copy.copy(self);

    def friction(self, qd):
        """
        Compute friction torque for joint rate C{qd}.
        Depending on fields in the Link object viscous and/or Coulomb friction
        are computed.
        
        @type qd: number
        @param qd: joint rate
        @rtype: number
        @return: joint friction torque
        """
        tau = 0.0
        if isinstance(qd, (np.ndarray, np.matrix)):
                qd = qd.flatten().T
        if self.B == None:
            self.B = 0
        tau = self.B * qd
        if self.Tc == None:
            self.Tc = np.mat([0,0])
        tau = tau + (qd > 0) * self.Tc[0,0] + (qd < 0) * self.Tc[0,1]
        return tau
        
    def nofriction(self, all=False):
        """
        Return a copy of the Link object with friction parameters set to zero.
        
        @type all: boolean
        @param all: if True then also zero viscous friction
        @rtype: Link
        @return: Copy of original Link object with zero friction
        @see: L{robot.nofriction}
        """

        l2 = self.copy()

        l2.Tc = np.array([0, 0])
        if all:
            l2.B = 0
        return l2;


# methods to set kinematic or dynamic parameters

    fields = ["alpha", "A", "theta", "D", "sigma", "offset", "m", "Jm", "G", "B", "convention"];
    
    def __setattr__(self, name, value):
        """
        Set attributes of the Link object
        
            - alpha; scalar
            - A; scalar
            - theta; scalar
            - D; scalar
            - sigma; scalar
            - offset; scalar
            - m; scalar
            - Jm; scalar
            - G; scalar
            - B; scalar
            - r; 3-vector
            - I; 3x3 matrix, 3-vector or 6-vector
            - Tc; scalar or 2-vector
            - qlim; 2-vector
        
        Inertia, I, can be specified as:
            - 3x3 inertia tensor
            - 3-vector, the diagonal of the inertia tensor
            - 6-vector, the unique elements of the inertia tensor [Ixx Iyy Izz Ixy Iyz Ixz]
            
        Coloumb friction, Tc, can be specifed as:
            - scalar, for the symmetric case when Tc- = Tc+
            - 2-vector, the assymetric case [Tc- Tc+]
            
        Joint angle limits, qlim, is a 2-vector giving the lower and upper limits
        of motion.
        """
    
        if value == None:
            self.__dict__[name] = value;
            return;
            
        if name in self.fields:
            # scalar parameter
            if isinstance(value, (np.ndarray,np.matrix)) and value.shape != (1,1):
                raise ValueError("Scalar required")
            if not isinstance(value, (int,float,np.int32,np.float64)):
                raise ValueError
            self.__dict__[name] = value

        elif name == "r":
            r = arg2array(value);
            if len(r) != 3:
                raise ValueError("matrix required")

            self.__dict__[name] = np.mat(r)
            
        elif name == "I":
            if isinstance(value, np.matrix) and value.shape == (3,3):
                self.__dict__[name] = value;
            else:
                v = arg2array(value);
                if len(v) == 3:
                    self.__dict__[name] = np.mat(diag(v))
                elif len(v) == 6:
                    self.__dict__[name] = np.mat([
                        [v[0],v[3],v[5]],
                        [v[3],v[1],v[4]],
                        [v[5],v[4],v[2]]])
                else:
                    raise ValueError("matrix required")

        elif name == "Tc":
            v = arg2array(value)
            
            if len(v) == 1:
                self.__dict__[name] =  np.mat([-v[0], v[0]])
            elif len(v) == 2:
                self.__dict__[name] = np.mat(v)
            else:
                raise ValueError;

        elif name == "qlim":
            v = arg2array(value);
            if len(v) == 2:
                self.__dict__[name] = np.mat(v);
            else:
                raise ValueError
        else:
            raise NameError("Unknown attribute <%s> of link" % name)


#   LINK.islimit(q) return if limit is exceeded: -1, 0, +1
    def islimit(self,q):
        """
        Check if joint limits exceeded.  Returns
            - -1 if C{q} is less than the lower limit
            - 0 if C{q} is within the limits
            - +1 if C{q} is greater than the high limit
        
        @type q: number
        @param q: Joint coordinate
        @rtype: -1, 0, +1
        @return: joint limit status
        """
        if not self.qlim:
            return 0

        return (q > self.qlim[1,0]) - (q < self.qlim[0,0])

    def tr(self, q):
        """
        Compute the transformation matrix for this link.  This is a function
        of kinematic parameters, the kinematic model (DH or MDH) and the joint
        coordinate C{q}.
        
        @type q: number
        @param q: joint coordinate
        @rtype: homogeneous transformation
        @return: Link transform M{A(q)}
        """
        
        an = self.A
        dn = self.D
        theta = self.theta


        if self.sigma == 0:
            theta = q   # revolute
        else:
            dn = q      # prismatic

        #print ('theta ',theta)
        theta += self.offset # adding offset
        #print ('theta offs' , theta)

        sa = np.sin(self.alpha); ca = np.cos(self.alpha);
        st = np.sin(theta); ct = np.cos(theta);

        #print (sa,ca,st,ct)

        if self.convention == Link.LINK_DH:
            # standard
            t =   np.mat([[ ct,    -st*ca, st*sa,  an*ct],
                    [st,    ct*ca,  -ct*sa, an*st],
                    [0, sa, ca, dn],
                    [0, 0,  0,  1]]);

        else:
            # modified
            t =   np.mat([[ ct,    -st,    0,  an],
                [st*ca, ct*ca,  -sa,    -sa*dn],
                [st*sa, ct*sa,  ca, ca*dn],
                [0, 0,  0,  1]]);

        #print (t)

        return t;

# transform.py

def rotx(theta):
    """
    Rotation about X-axis
    
    @type theta: number
    @param theta: the rotation angle
    @rtype: 3x3 orthonormal matrix
    @return: rotation about X-axis

    @see: L{roty}, L{rotz}, L{rotvec}
    """
    
    ct = np.cos(theta)
    st = np.sin(theta)
    return np.mat([[1,  0,    0],
            [0,  ct, -st],
            [0,  st,  ct]])

def roty(theta):
    """
    Rotation about Y-axis
    
    @type theta: number
    @param theta: the rotation angle
    @rtype: 3x3 orthonormal matrix
    @return: rotation about Y-axis

    @see: L{rotx}, L{rotz}, L{rotvec}
    """
    
    ct = np.cos(theta)
    st = np.sin(theta)

    return np.mat([[ct,   0,   st],
            [0,    1,    0],
            [-st,  0,   ct]])

def rotz(theta):
    """
    Rotation about Z-axis
    
    @type theta: number
    @param theta: the rotation angle
    @rtype: 3x3 orthonormal matrix
    @return: rotation about Z-axis

    @see: L{rotx}, L{roty}, L{rotvec}
    """
    
    ct = np.cos(theta)
    st = np.sin(theta)

    return np.mat([[ct,      -st,  0],
            [st,       ct,  0],
            [ 0,    0,  1]])

def trotx(theta):
    """
    Rotation about X-axis
    
    @type theta: number
    @param theta: the rotation angle
    @rtype: 4x4 homogeneous matrix
    @return: rotation about X-axis

    @see: L{troty}, L{trotz}, L{rotx}
    """
    return r2t(rotx(theta))

def troty(theta):
    """
    Rotation about Y-axis
    
    @type theta: number
    @param theta: the rotation angle
    @rtype: 4x4 homogeneous matrix
    @return: rotation about Y-axis

    @see: L{troty}, L{trotz}, L{roty}
    """
    return r2t(roty(theta))

def trotz(theta):
    """
    Rotation about Z-axis
    
    @type theta: number
    @param theta: the rotation angle
    @rtype: 4x4 homogeneous matrix
    @return: rotation about Z-axis

    @see: L{trotx}, L{troty}, L{rotz}
    """
    return r2t(rotz(theta))


##################### Euler angles

def tr2eul(m):
    """
    Extract Euler angles.
    Returns a vector of Euler angles corresponding to the rotational part of 
    the homogeneous transform.  The 3 angles correspond to rotations about
    the Z, Y and Z axes respectively.
    
    @type m: 3x3 or 4x4 matrix
    @param m: the rotation matrix
    @rtype: 1x3 matrix
    @return: Euler angles [S{theta} S{phi} S{psi}]
    
    @see:  L{eul2tr}, L{tr2rpy}
    """
    
    try:
        m = np.mat(m)
        if ishomog(m):
            euler = np.mat(np.zeros((1,3)))
            if np.linalg.norm(m[0,2])<finfo(float).eps and np.linalg.norm(m[1,2])<finfo(float).eps:
                # singularity
                euler[0,0] = 0
                sp = 0
                cp = 1
                euler[0,1] = arctan2(cp*m[0,2] + sp*m[1,2], m[2,2])
                euler[0,2] = arctan2(-sp*m[0,0] + cp*m[1,0], -sp*m[0,1] + cp*m[1,1])
                return euler
            else:
                euler[0,0] = arctan2(m[1,2],m[0,2])
                sp = np.sin(euler[0,0])
                cp = np.cos(euler[0,0])
                euler[0,1] = arctan2(cp*m[0,2] + sp*m[1,2], m[2,2])
                euler[0,2] = arctan2(-sp*m[0,0] + cp*m[1,0], -sp*m[0,1] + cp*m[1,1])
                return euler
            
    except ValueError:
        euler = []
        for i in range(0,len(m)):
            euler.append(tr2eul(m[i]))
        return euler
        

def eul2r(phi, theta=None, psi=None):
    """
    Rotation from Euler angles.
    
    Two call forms:
        - R = eul2r(S{theta}, S{phi}, S{psi})
        - R = eul2r([S{theta}, S{phi}, S{psi}])
    These correspond to rotations about the Z, Y, Z axes respectively.

    @type phi: number or list/array/matrix of angles
    @param phi: the first Euler angle, or a list/array/matrix of angles
    @type theta: number
    @param theta: the second Euler angle
    @type psi: number
    @param psi: the third Euler angle
    @rtype: 3x3 orthonormal matrix
    @return: R([S{theta} S{phi} S{psi}])

    @see:  L{tr2eul}, L{eul2tr}, L{tr2rpy}

    """

    n = 1
    if theta == None and psi==None:
        # list/array/matrix argument
        phi = np.mat(phi)
        if numcols(phi) != 3:
            error('bad arguments')
        else:
            n = numrows(phi)
            psi = phi[:,2]
            theta = phi[:,1]
            phi = phi[:,0]
    elif (theta!=None and psi==None) or (theta==None and psi!=None):
        error('bad arguments')
    elif not isinstance(phi,(int,np.int32,float,np.float64)):
        # all args are vectors
        phi = np.mat(phi)
        n = numrows(phi)
        theta = np.mat(theta)
        psi = np.mat(psi)

    if n>1:
        R = []
        for i in range(0,n):
                r = rotz(phi[i,0]) * roty(theta[i,0]) * rotz(psi[i,0])
                R.append(r)
        return R
    try:
        r = rotz(phi[0,0]) * roty(theta[0,0]) * rotz(psi[0,0])
        return r
    except:
        r = rotz(phi) * roty(theta) * rotz(psi)
        return r

def eul2tr(phi,theta=None,psi=None):
    """
    Rotation from Euler angles.
    
    Two call forms:
        - R = eul2tr(S{theta}, S{phi}, S{psi})
        - R = eul2tr([S{theta}, S{phi}, S{psi}])
    These correspond to rotations about the Z, Y, Z axes respectively.

    @type phi: number or list/array/matrix of angles
    @param phi: the first Euler angle, or a list/array/matrix of angles
    @type theta: number
    @param theta: the second Euler angle
    @type psi: number
    @param psi: the third Euler angle
    @rtype: 4x4 homogenous matrix
    @return: R([S{theta} S{phi} S{psi}])

    @see:  L{tr2eul}, L{eul2r}, L{tr2rpy}

    """
    return r2t( eul2r(phi, theta, psi) )


################################## RPY angles


def tr2rpy(m):
    """
    Extract RPY angles.
    Returns a vector of RPY angles corresponding to the rotational part of 
    the homogeneous transform.  The 3 angles correspond to rotations about
    the Z, Y and X axes respectively.
    
    @type m: 3x3 or 4x4 matrix
    @param m: the rotation matrix
    @rtype: 1x3 matrix
    @return: RPY angles [S{theta} S{phi} S{psi}]
    
    @see:  L{rpy2tr}, L{tr2eul}
    """
    try:
        m = np.mat(m)
        if ishomog(m):
            rpy = np.mat(np.zeros((1,3)))
            if np.linalg.norm(m[0,0])<finfo(float).eps and np.linalg.norm(m[1,0])<finfo(float).eps:
                # singularity
                rpy[0,0] = 0
                rpy[0,1] = arctan2(-m[2,0], m[0,0])
                rpy[0,2] = arctan2(-m[1,2], m[1,1])
                return rpy
            else:
                rpy[0,0] = arctan2(m[1,0],m[0,0])
                sp = np.sin(rpy[0,0])
                cp = np.cos(rpy[0,0])
                rpy[0,1] = arctan2(-m[2,0], cp*m[0,0] + sp*m[1,0])
                rpy[0,2] = arctan2(sp*m[0,2] - cp*m[1,2], cp*m[1,1] - sp*m[0,1])
                return rpy
            
    except ValueError:
        rpy = []
        for i in range(0,len(m)):
            rpy.append(tr2rpy(m[i]))
        return rpy
        
def rpy2r(roll, pitch=None,yaw=None):
    """
    Rotation from RPY angles.
    
    Two call forms:
        - R = rpy2r(S{theta}, S{phi}, S{psi})
        - R = rpy2r([S{theta}, S{phi}, S{psi}])
    These correspond to rotations about the Z, Y, X axes respectively.

    @type roll: number or list/array/matrix of angles
    @param roll: roll angle, or a list/array/matrix of angles
    @type pitch: number
    @param pitch: pitch angle
    @type yaw: number
    @param yaw: yaw angle
    @rtype: 4x4 homogenous matrix
    @return: R([S{theta} S{phi} S{psi}])

    @see:  L{tr2rpy}, L{rpy2r}, L{tr2eul}

    """
    n=1
    if pitch==None and yaw==None:
        roll= np.mat(roll)
        if numcols(roll) != 3:
            error('bad arguments')
        n = numrows(roll)
        pitch = roll[:,1]
        yaw = roll[:,2]
        roll = roll[:,0]
    if n>1:
        R = []
        for i in range(0,n):
            r = rotz(roll[i,0]) * roty(pitch[i,0]) * rotx(yaw[i,0])
            R.append(r)
        return R
    try:
        r = rotz(roll[0,0]) * roty(pitch[0,0]) * rotx(yaw[0,0])
        return r
    except:
        r = rotz(roll) * roty(pitch) * rotx(yaw)
        return r


def rpy2tr(roll, pitch=None, yaw=None):
    """
    Rotation from RPY angles.
    
    Two call forms:
        - R = rpy2tr(r, p, y)
        - R = rpy2tr([r, p, y])
    These correspond to rotations about the Z, Y, X axes respectively.

    @type roll: number or list/array/matrix of angles
    @param roll: roll angle, or a list/array/matrix of angles
    @type pitch: number
    @param pitch: pitch angle
    @type yaw: number
    @param yaw: yaw angle
    @rtype: 4x4 homogenous matrix
    @return: R([S{theta} S{phi} S{psi}])

    @see:  L{tr2rpy}, L{rpy2r}, L{tr2eul}

    """
    return r2t( rpy2r(roll, pitch, yaw) )

###################################### OA vector form


def oa2r(o,a):
    """Rotation from 2 vectors.
    The matrix is formed from 3 vectors such that::
        R = [N O A] and N = O x A.  

    In robotics A is the approach vector, along the direction of the robot's 
    gripper, and O is the orientation vector in the direction between the 
    fingertips.
    
    The submatrix is guaranteed to be orthonormal so long as O and A are 
    not parallel.
    
    @type o: 3-vector
    @param o: The orientation vector.
    @type a: 3-vector
    @param a: The approach vector
    @rtype: 3x3 orthonormal rotation matrix
    @return: Rotatation matrix
    
    @see: L{rpy2r}, L{eul2r}
    """
    n = crossp(o, a)
    n = unit(n)
    o = crossp(a, n);
    o = unit(o).reshape(3,1)
    a = unit(a).reshape(3,1)
    return bmat('n o a')


def oa2tr(o,a):
    """otation from 2 vectors.
    The rotation submatrix is formed from 3 vectors such that::

        R = [N O A] and N = O x A.  

    In robotics A is the approach vector, along the direction of the robot's 
    gripper, and O is the orientation vector in the direction between the 
    fingertips.
    
    The submatrix is guaranteed to be orthonormal so long as O and A are 
    not parallel.
    
    @type o: 3-vector
    @param o: The orientation vector.
    @type a: 3-vector
    @param a: The approach vector
    @rtype: 4x4 homogeneous transformation matrix
    @return: Transformation matrix
    
    @see: L{rpy2tr}, L{eul2tr}
    """
    return r2t(oa2r(o,a))
    
    
###################################### angle/vector form


def rotvec2r(theta, v):
    """
    Rotation about arbitrary axis.  Compute a rotation matrix representing
    a rotation of C{theta} about the vector C{v}.
    
    @type v: 3-vector
    @param v: rotation vector
    @type theta: number
    @param theta: the rotation angle
    @rtype: 3x3 orthonormal matrix
    @return: rotation

    @see: L{rotx}, L{roty}, L{rotz}
    """
    v = arg2array(v);
    ct = np.cos(theta)
    st = np.sin(theta)
    vt = 1-ct
    r = np.mat([[ct,         -v[2]*st,    v[1]*st],\
             [v[2]*st,          ct,   -v[0]*st],\
             [-v[1]*st,  v[0]*st,           ct]])
    return v*v.T*vt+r

def rotvec2tr(theta, v):
    """
    Rotation about arbitrary axis.  Compute a rotation matrix representing
    a rotation of C{theta} about the vector C{v}.
    
    @type v: 3-vector
    @param v: rotation vector
    @type theta: number
    @param theta: the rotation angle
    @rtype: 4x4 homogeneous matrix
    @return: rotation

    @see: L{trotx}, L{troty}, L{trotz}
    """
    return r2t(rotvec2r(theta, v))


###################################### translational transform


def transl(x, y=None, z=None):
    """
    Create or decompose translational homogeneous transformations.
    
    Create a homogeneous transformation
    ===================================
    
        - T = transl(v)
        - T = transl(vx, vy, vz)
        
        The transformation is created with a unit rotation submatrix.
        The translational elements are set from elements of v which is
        a list, array or matrix, or from separate passed elements.
    
    Decompose a homogeneous transformation
    ======================================
    

        - v = transl(T)   
    
        Return the translation vector
    """
           
    if y==None and z==None:
            x=np.mat(x)
            try:
                    if ishomog(x):
                            return x[0:3,3].reshape(3,1)
                    else:
                            return np.concatenate((np.concatenate((np.eye(3),x.reshape(3,1)),1),np.mat([0,0,0,1])))
            except AttributeError:
                    n=len(x)
                    r = [[],[],[]]
                    for i in range(n):
                            r = np.concatenate((r,x[i][0:3,3]),1)
                    return r
    elif y!=None and z!=None:
            return np.concatenate((np.concatenate((np.eye(3),np.mat([x,y,z]).T),1),np.mat([0,0,0,1])))


###################################### Skew symmetric transform


def skew(*args):
    """
    Convert to/from skew-symmetric form.  A skew symmetric matrix is a matrix
    such that M = -M'
    
    Two call forms
    
        -ss = skew(v)
        -v = skew(ss)
        
    The first form builds a 3x3 skew-symmetric from a 3-element vector v.
    The second form takes a 3x3 skew-symmetric matrix and returns the 3 unique
    elements that it contains.
    
    """
    
    def ss(b):
        return  np.matrix([
            [0, -b[2],  b[1]],
            [b[2],  0,  -b[0]],
            [-b[1], b[0],   0]]);

    if len(args) == 1:
        # convert matrix to skew vector
        b = args[0];
        
        if isrot(b):
            return 0.5*np.matrix( [b[2,1]-b[1,2], b[0,2]-b[2,0], b[1,0]-b[0,1]] );
        elif ishomog(b):
            return vstack( (b[0:3,3], 0.5*np.matrix( [b[2,1]-b[1,2], b[0,2]-b[2,0], b[1,0]-b[0,1]] ).T) );

    
    # build skew-symmetric matrix
          
        b = arg2array(b);
        if len(b) == 3:
            return ss(b);
        elif len(b) == 6:
            r = hstack( (ss(b[3:6]), np.mat(b[0:3]).T) );
            r = vstack( (r, np.mat([0, 0, 0, 1])) );
            return r;
            
    elif len(args) == 3:
            return ss(args);    
    elif len(args) == 6:
            r = hstack( (ss(args[3:6]), np.mat(args[0:3]).T) );
            r = vstack( (r, np.mat([0, 0, 0, 1])) );
            return r;    
    else:
        raise ValueError;



def tr2diff(t1, t2):
    """
    Convert a transform difference to differential representation.
    Returns the 6-element differential motion required to move
    from T1 to T2 in base coordinates.
    
    @type t1: 4x4 homogeneous transform
    @param t1: Initial value
    @type t2: 4x4 homogeneous transform
    @param t2: Final value
    @rtype: 6-vector
    @return: Differential motion [dx dy dz drx dry drz]
    @see: L{skew}
    """
    
    t1 = np.mat(t1)
    t2 = np.mat(t2)
    
    d = np.concatenate(
        (t2[0:3,3]-t1[0:3,3],
        0.5*(   crossp(t1[0:3,0], t2[0:3,0]) +
                crossp(t1[0:3,1], t2[0:3,1]) +
                crossp(t1[0:3,2], t2[0:3,2]) )
            ))
    return d

################################## Utility


def trinterp(T0, T1, r):
    """
    Interpolate homogeneous transformations.
    Compute a homogeneous transform interpolation between C{T0} and C{T1} as
    C{r} varies from 0 to 1 such that::
    
        trinterp(T0, T1, 0) = T0
        trinterp(T0, T1, 1) = T1
        
    Rotation is interpolated using quaternion spherical linear interpolation.

    @type T0: 4x4 homogeneous transform
    @param T0: Initial value
    @type T1: 4x4 homogeneous transform
    @param T1: Final value
    @type r: number
    @param r: Interpolation index, in the range 0 to 1 inclusive
    @rtype: 4x4 homogeneous transform
    @return: Interpolated value
    @see: L{quaternion}, L{ctraj}
    """
    
    q0 = quaternion(T0)
    q1 = quaternion(T1)
    p0 = transl(T0)
    p1 = transl(T1)

    qr = q0.interp(q1, r)
    pr = p0*(1-r) + r*p1

    return vstack( (np.concatenate((qr.r(),pr),1), np.mat([0,0,0,1])) )


def trnorm(t):
    """
    Normalize a homogeneous transformation.
    Finite word length arithmetic can cause transforms to become `unnormalized',
    that is the rotation submatrix is no longer orthonormal (det(R) != 1).
    
    The rotation submatrix is re-orthogonalized such that the approach vector
    (third column) is unchanged in direction::
    
        N = O x A
        O = A x N

    @type t: 4x4 homogeneous transformation
    @param t: the transform matrix to convert
    @rtype: 3x3 orthonormal rotation matrix
    @return: rotation submatrix
    @see: L{oa2tr}
    @bug: Should work for 3x3 matrix as well.
    """

    t = np.mat(t)      # N O A
    n = crossp(t[0:3,1],t[0:3,2]) # N = O X A
    o = crossp(t[0:3,2],t[0:3,0]) # O = A x N
    return np.concatenate(( np.concatenate((unit(n),unit(t[0:3,1]),unit(t[0:3,2]),t[0:3,3]),1),
         np.mat([0,0,0,1])))


def t2r(T):
    """
    Return rotational submatrix of a homogeneous transformation.
    @type T: 4x4 homogeneous transformation
    @param T: the transform matrix to convert
    @rtype: 3x3 orthonormal rotation matrix
    @return: rotation submatrix
    """    
    
    if ishomog(T)==False:
        error( 'input must be a homogeneous transform')
    return T[0:3,0:3]


def r2t(R):
    """
    Convert a 3x3 orthonormal rotation matrix to a 4x4 homogeneous transformation::
    
        T = | R 0 |
            | 0 1 |
            
    @type R: 3x3 orthonormal rotation matrix
    @param R: the rotation matrix to convert
    @rtype: 4x4 homogeneous matrix
    @return: homogeneous equivalent
    """
    
    return np.concatenate( (np.concatenate( (R, np.zeros((3,1))),1), np.mat([0,0,0,1])) )

#  Quaternion.py

class quaternion:
    """A quaternion is a compact method of representing a 3D rotation that has
    computational advantages including speed and numerical robustness.

    A quaternion has 2 parts, a scalar s, and a vector v and is typically written::

        q = s <vx vy vz>

    A unit quaternion is one for which M{s^2+vx^2+vy^2+vz^2 = 1}.

    A quaternion can be considered as a rotation about a vector in space where
    q = cos (theta/2) sin(theta/2) <vx vy vz>
    where <vx vy vz> is a unit vector.

    Various functions such as INV, NORM, UNIT and PLOT are overloaded for
    quaternion objects.

    Arithmetic operators are also overloaded to allow quaternion multiplication,
    division, exponentiaton, and quaternion-vector multiplication (rotation).
    """

    def __init__(self, *args):
        '''
Constructor for quaternion objects:

    - q = quaternion(v, theta)    from vector plus angle
    - q = quaternion(R)       from a 3x3 or 4x4 matrix
    - q = quaternion(q)       from another quaternion
    - q = quaternion(s)       from a scalar
    - q = quaternion(v)       from a matrix/array/list v = [s v1 v2 v3]
    - q = quaternion(s, v1, v2, v3)    from 4 elements
    - q = quaternion(s, v)    from 4 elements
'''

        self.vec = [];
        
        if len(args) == 0:
                # default is a null rotation
                self.s = 1.0
                self.v = np.matrix([0.0, 0.0, 0.0])

        elif len(args) == 1:
            arg = args[0]
            
            if isinstance(arg, quaternion):
            # Q = QUATERNION(q) from another quaternion
                self.s = arg.s
                self.v = arg.v
                return
                
            if type(arg) is np.matrix:
                # Q = QUATERNION(R) from a 3x3
                if (arg.shape == (3,3)):
                    self.tr2q(arg);
                    return;

                # Q = QUATERNION(R) from a 4x4
                if (arg.shape == (4,4)):
                    self.tr2q(arg[0:3,0:3]);
                    return;
                    
            # some kind of list, vector, scalar...
            
            v = arg2array(arg);
            
            if len(v) == 4:
                # Q = QUATERNION([s v1 v2 v3]) from 4 elements
                self.s = v[0];
                self.v = np.mat(v[1:4]);
                
            elif len(v) == 3:
                self.s = 0
                self.v = np.mat(v[0:3]);

            elif len(v) == 1:
                # Q = QUATERNION(s) from a scalar
                #Q = QUATERNION(s)               from a scalar
                self.s = v[0];
                self.v = np.mat([0, 0, 0])

        elif len(args) == 2:
            # Q = QUATERNION(v, theta) from vector plus angle
            # Q = quaternion(s, v);

            a1 = arg2array(args[0]);
            a2 = arg2array(args[1]);
            
            if len(a1) == 1 and len(a2) == 3:
                # s, v
                self.s = a1[0];
                self.v = np.mat(a2);
            elif len(a1) == 3 and len(a2) == 1:
                # v, theta
                self.s = a2[0];
                self.v = np.mat(a1);

        elif len(args) == 4:
            self.s = args[0];
            self.v = np.mat(args[1:4])

        else:
                print ("error")
                return None

    def __repr__(self):
            return "%f <%f, %f, %f>" % (self.s, self.v[0,0], self.v[0,1], self.v[0,2])


    def tr2q(self, t):
        #TR2Q   Convert homogeneous transform to a unit-quaternion
        #
        #   Q = tr2q(T)
        #
        #   Return a unit quaternion corresponding to the rotational part of the
        #   homogeneous transform T.

        qs = sqrt(trace(t)+1)/2.0
        kx = t[2,1] - t[1,2]    # Oz - Ay
        ky = t[0,2] - t[2,0]    # Ax - Nz
        kz = t[1,0] - t[0,1]    # Ny - Ox

        if (t[0,0] >= t[1,1]) and (t[0,0] >= t[2,2]):
                kx1 = t[0,0] - t[1,1] - t[2,2] + 1      # Nx - Oy - Az + 1
                ky1 = t[1,0] + t[0,1]           # Ny + Ox
                kz1 = t[2,0] + t[0,2]           # Nz + Ax
                add = (kx >= 0)
        elif (t[1,1] >= t[2,2]):
                kx1 = t[1,0] + t[0,1]           # Ny + Ox
                ky1 = t[1,1] - t[0,0] - t[2,2] + 1  # Oy - Nx - Az + 1
                kz1 = t[2,1] + t[1,2]           # Oz + Ay
                add = (ky >= 0)
        else:
                kx1 = t[2,0] + t[0,2]           # Nz + Ax
                ky1 = t[2,1] + t[1,2]           # Oz + Ay
                kz1 = t[2,2] - t[0,0] - t[1,1] + 1  # Az - Nx - Oy + 1
                add = (kz >= 0)

        if add:
                kx = kx + kx1
                ky = ky + ky1
                kz = kz + kz1
        else:
                kx = kx - kx1
                ky = ky - ky1
                kz = kz - kz1

        kv = np.matrix([kx, ky, kz])
        nm = np.linalg.norm( kv )
        if nm == 0:
                self.s = 1.0
                self.v = np.matrix([0.0, 0.0, 0.0])

        else:
                self.s = qs
                self.v = (sqrt(1 - qs**2) / nm) * kv

    ############### OPERATORS #########################################
    #PLUS Add two quaternion objects
    #
    # Invoked by the + operator
    #
    # q1+q2 standard quaternion addition
    def __add__(self, q):
        '''
        Return a new quaternion that is the element-wise sum of the operands.
        '''
        if isinstance(q, quaternion):
            qr = quaternion()
            qr.s = 0

            qr.s = self.s + q.s
            qr.v = self.v + q.v

            return qr
        else:
            raise ValueError

    #MINUS Subtract two quaternion objects
    #
    # Invoked by the - operator
    #
    # q1-q2 standard quaternion subtraction

    def __sub__(self, q):
        '''
        Return a new quaternion that is the element-wise difference of the operands.
        '''
        if isinstance(q, quaternion):
            qr = quaternion()
            qr.s = 0

            qr.s = self.s - q.s
            qr.v = self.v - q.v

            return qr
        else:
            raise ValueError

    # q * q  or q * const
    def __mul__(self, q2):
        '''
        Quaternion product. Several cases are handled
        
            - q * q   quaternion multiplication
            - q * c   element-wise multiplication by constant
            - q * v   quaternion-vector multiplication q * v * q.inv();
        '''
        qr = quaternion();
        
        if isinstance(q2, quaternion):
                
            #Multiply unit-quaternion by unit-quaternion
            #
            #   QQ = qqmul(Q1, Q2)

            # decompose into scalar and vector components
            s1 = self.s;    v1 = self.v
            s2 = q2.s;  v2 = q2.v

            # form the product
            qr.s = s1*s2 - v1*v2.T
            qr.v = s1*v2 + s2*v1 + cross(v1,v2)

        elif type(q2) is np.matrix:
                
            # Multiply vector by unit-quaternion
            #
            #   Rotate the vector V by the unit-quaternion Q.

            if q2.shape == (1,3) or q2.shape == (3,1):
                    qr = self * quaternion(q2) * self.inv()
                    return qr.v;
            else:
                    raise ValueError;

        else:
            qr.s = self.s * q2
            qr.v = self.v * q2

        return qr

    def __rmul__(self, c):
        '''
        Quaternion product. Several cases are handled
 
            - c * q   element-wise multiplication by constant
        '''
        qr = quaternion()
        qr.s = self.s * c
        qr.v = self.v * c

        return qr
        
    def __imul__(self, x):
        '''
        Quaternion in-place multiplication
        
            - q *= q2
            
        '''
        
        if isinstance(x, quaternion):
            s1 = self.s;   
            v1 = self.v
            s2 = x.s
            v2 = x.v

            # form the product
            self.s = s1*s2 - v1*v2.T
            self.v = s1*v2 + s2*v1 + cross(v1,v2)

        elif isscalar(x):
            self.s *= x;
            self.v *= x;

        return self;


    def __div__(self, q):
        '''Return quaternion quotient.  Several cases handled:
            - q1 / q2      quaternion division implemented as q1 * q2.inv()
            - q1 / c       element-wise division
        '''
        if isinstance(q, quaternion):
            qr = quaternion()
            qr = self * q.inv()
        elif isscalar(q):
            qr.s = self.s / q
            qr.v = self.v / q

        return qr


    def __pow__(self, p):
        '''
        Quaternion exponentiation.  Only integer exponents are handled.  Negative
        integer exponents are supported.
        '''
        
        # check that exponent is an integer
        if not isinstance(p, int):
            raise ValueError
        
        qr = quaternion()
        q = quaternion(self);
        
        # multiply by itself so many times
        for i in range(0, abs(p)):
            qr *= q

        # if exponent was negative, invert it
        if p < 0:
            qr = qr.inv()

        return qr

    def copy(self):
        """
        Return a copy of the quaternion.
        """
        return copy.copy(self);
                
    def inv(self):
        """Return the inverse.
        
        @rtype: quaternion
        @return: the inverse
        """
        
        qi = quaternion(self);
        qi.v = -qi.v;
        
        return qi;



    def norm(self):
        """Return the norm of this quaternion.
        
        @rtype: number
        @return: the norm
        """
        
        return np.linalg.norm(self.double())

    def double(self):
        """Return the quaternion as 4-element vector.
        
        @rtype: 4-vector
        @return: the quaternion elements
        """
        return np.concatenate( (np.mat(self.s), self.v), 1 )


    def unit(self):
        """Return an equivalent unit quaternion
        
        @rtype: quaternion
        @return: equivalent unit quaternion
        """
        
        qr = quaternion()
        nm = self.norm()

        qr.s = self.s / nm
        qr.v = self.v / nm

        return qr


    def tr(self):
        """Return an equivalent rotation matrix.
        
        @rtype: 4x4 homogeneous transform
        @return: equivalent rotation matrix
        """

        return r2t( self.r() )

    def r(self):
        """Return an equivalent rotation matrix.
        
        @rtype: 3x3 orthonormal rotation matrix
        @return: equivalent rotation matrix
        """

        s = self.s;
        x = self.v[0,0]
        y = self.v[0,1]
        z = self.v[0,2]

        return np.matrix([[ 1-2*(y**2+z**2),   2*(x*y-s*z),    2*(x*z+s*y)],
                       [2*(x*y+s*z),    1-2*(x**2+z**2),    2*(y*z-s*x)],
                       [2*(x*z-s*y),    2*(y*z+s*x),    1-2*(x**2+y**2)]])

    

#QINTERP Interpolate rotations expressed by quaternion objects
#
#   QI = qinterp(Q1, Q2, R)
#
# Return a unit-quaternion that interpolates between Q1 and Q2 as R moves
# from 0 to 1.  This is a spherical linear interpolation (slerp) that can
# be interpretted as interpolation along a great circle arc on a sphere.
#
# If r is a vector, QI, is a cell array of quaternions, each element
# corresponding to sequential elements of R.
#
# See also: CTRAJ, QUATERNION.

# MOD HISTORY
# 2/99 convert to use of objects
# $Log: qinterp.m,v $
# Revision 1.3  2002/04/14 11:02:54  pic
# Changed see also line.
#
# Revision 1.2  2002/04/01 12:06:48  pic
# General tidyup, help comments, copyright, see also, RCS keys.
#
# $Revision: 1.3 $
#
# Copyright (C) 1999-2002, by Peter I. Corke

    def interp(Q1, Q2, r):
        q1 = Q1.double()
        q2 = Q2.double()

        theta = np.arccos(q1*q2.T)
        q = []
        count = 0

        if isscalar(r):
            if r<0 or r>1:
                raise 'R out of range'
            if theta == 0:
                q = quaternion(Q1)
            else:
                q = quaternion( (np.sin((1-r)*theta) * q1 + np.sin(r*theta) * q2) / np.sin(theta) )
        else:
            for R in r:
                if theta == 0:
                    qq = Q1
                else:
                    qq = quaternion( (np.sin((1-R)*theta) * q1 + np.sin(R*theta) * q2) / np.sin(theta) )
                q.append(qq)
        return q

# utility.py

def ishomog(tr):
    """
    True if C{tr} is a 4x4 homogeneous transform.
    
    @note: Only the dimensions are tested, not whether the rotation submatrix
    is orthonormal.
    
    @rtype: boolean
    """
    
    return tr.shape == (4,4)



def isrot(r):
    """
    True if C{tr} is a 3x3 matrix.
    
    @note: Only the dimensions are tested, not whether the matrix
    is orthonormal.
    
    @rtype: boolean
    """
    return r.shape == (3,3)


def isvec(v, l=3):
    """
    True if C{tr} is an l-vector.  
    
    @param v: object to test
    @type l: integer
    @param l: length of vector (default 3)
   
    @rtype: boolean
    """
    return v.shape == (l,1) or v.shape == (1,l) or v.shape == (l,)


def numcols(m):
    """
    Number of columns in a matrix.
    
    @type m: matrix
    @return: the number of columns in the matrix.
    return m.shape[1];
    """
    return m.shape[1];
    
def numrows(m):
    """
    Number of rows in a matrix.
    
    @type m: matrix
    @return: the number of rows in the matrix.
    return m.shape[1];
    """
    return m.shape[0];

################ vector operations

def unit(v):
    """
    Unit vector.
    
    @type v: vector
    @rtype: vector
    @return: unit-vector parallel to C{v}
    """
    return np.mat(v / np.linalg.norm(v))
    
def crossp(v1, v2):
    """
    Vector cross product.
    
    @note Differs from L{numpy.cross} in that vectors can be row or
    column.
    
    @type v1: 3-vector
    @type v2: 3-vector
    @rtype: 3-vector
    @return: Cross product M{v1 x v2}
    """
    v1=np.mat(v1)
    v2=np.mat(v2)
    v1=v1.reshape(3,1)
    v2=v2.reshape(3,1)
    v = np.matrix(np.zeros( (3,1) ))
    v[0] = v1[1]*v2[2] - v1[2]*v2[1]
    v[1] = v1[2]*v2[0] - v1[0]*v2[2]
    v[2] = v1[0]*v2[1] - v1[1]*v2[0]
    return v

################ misc support functions

def arg2array(arg):
    """
    Convert a 1-dimensional argument that is either a list, array or matrix to an 
    array.
    
    Useful for functions where the argument might be in any of these formats:::
            func(a)
            func(1,2,3)
            
            def func(*args):
                if len(args) == 1:
                    v = arg2array(arg[0]);
                elif len(args) == 3:
                    v = arg2array(args);
             .
             .
             .
    
    @rtype: array
    @return: Array equivalent to C{arg}.
    """
    if isinstance(arg, (np.matrix, np.ndarray)):
        s = arg.shape;
        if len(s) == 1:
            return np.array(arg);
        if min(s) == 1:
            return np.array(arg).flatten();

    elif isinstance(arg, list):
        return np.array(arg);

    elif isinstance(arg, (int, float, float32, np.float64)):
        return np.array([arg]);
        
    raise ValueError;
        


import traceback;

def error(s):
    """
    Common error handler.  Display the error string, execute a traceback then raise
    an execption to return to the interactive prompt.
    """
    print ('Robotics toolbox error:', s)

    #traceback.print_exc();
    raise ValueError
    

# common.py

def h2e(P):
    """
    Convert point coordinates from homogeneous to Euclidian
    Homogenous coordinates are in a column vector
    """
    P1=np.transpose(P)
    P1=np.asarray(P1)[0]
    return P1[0:np.size(P1)-1]

def e2h(P):
    """
    Convert point coordinates from Euclidian to homogeneous
    """
    return np.append(P,1.0)

def trpt(T,P0):
    """"
    Transform point P0 with homogeneous matrix T
    """
    Ph0=np.transpose(np.asmatrix(e2h(P0)))
    Ph1=T*Ph0
    return h2e(Ph1)


# kinematics.py (without ikine for puma560)

def fkine(robot, q):
    """
    Computes the forward kinematics for each joint space point defined by C{q}.
    ROBOT is a robot object.

    For an n-axis manipulator C{q} is an n element vector or an m x n matrix of
    robot joint coordinates.

    If C{q} is a vector it is interpretted as the generalized joint coordinates, and
    C{fkine} returns a 4x4 homogeneous transformation for the tool of
    the manipulator.

    If C{q} is a matrix, the rows are interpretted as the generalized 
    joint coordinates for a sequence of points along a trajectory.  q[i,j] is
    the j'th joint parameter for the i'th trajectory point.  In this case
    C{fkine} returns a list of matrices for each point
    along the path.

    The robot's base or tool transform, if present, are incorporated into the
    result.
    
    @type robot: Robot instance
    @param robot: The robot
    @type q: vector
    @param q: joint coordinate
    @see: L{Link}, L{Robot}, L{ikine}
    """


    q = np.mat(q)
    n = robot.n
    #print n,numrows(q),numcols(q)
    if numrows(q)==1 and numcols(q)==n:
        t = robot.base
        for i in range(0,n):
            t = t * robot.links[i].tr(q[0,i])
        t = t * robot.tool
        #print numrows(t), numcols(t)
        return t
    else:
        if numcols(q) != n:
            raise 'bad data'
        t = []
        for qv in q:        # for each trajectory point
            tt = robot.base
            for i in range(0,n):
                tt = tt * robot.links[i].tr(qv[0,i])
            t.append(tt*robot.tool)
        return t



def ikine(robot, tr, q=None, m=None):
    """
    Inverse manipulator kinematics.
    Computes the joint coordinates corresponding to the end-effector transform C{tr}.
    Typically invoked as

        - Q = IKINE(ROBOT, T)
        - Q = IKINE(ROBOT, T, Q)
        - Q = IKINE(ROBOT, T, Q, M)

    Uniqueness
    ==========
    Note that the inverse kinematic solution is generally not unique, and 
    depends on the initial guess C{q} (which defaults to 0).

    Iterative solution
    ==================
    Solution is computed iteratively using the pseudo-inverse of the
    manipulator Jacobian.

    Such a solution is completely general, though much less efficient 
    than specific inverse kinematic solutions derived symbolically.

    This approach allows a solution to obtained at a singularity, but 
    the joint angles within the null space are arbitrarily assigned.

    Operation on a trajectory
    =========================
    If C{tr} is a list of transforms (a trajectory) then the solution is calculated
    for each transform in turn.  The return values is a matrix with one row for each
    input transform.  The initial estimate for the iterative solution at 
    each time step is taken as the solution from the previous time step.

    Fewer than 6DOF
    ===============
    If the manipulator has fewer than 6 DOF then this method of solution
    will fail, since the solution space has more dimensions than can
    be spanned by the manipulator joint coordinates.  In such a case
    it is necessary to provide a mask matrix, C{m}, which specifies the 
    Cartesian DOF (in the wrist coordinate frame) that will be ignored
    in reaching a solution.  The mask matrix has six elements that
    correspond to translation in X, Y and Z, and rotation about X, Y and
    Z respectively.  The value should be 0 (for ignore) or 1.  The number
    of non-zero elements should equal the number of manipulator DOF.

    For instance with a typical 5 DOF manipulator one would ignore
    rotation about the wrist axis, that is, M = [1 1 1 1 1 0].


    @type robot: Robot instance
    @param robot: The robot
    @type tr: homgeneous transformation
    @param tr: End-effector pose
    @type q: vector
    @param q: initial estimate of joint coordinate
    @type m: vector
    @param m: mask vector
    @rtype: vector
    @return: joint coordinate
    @see: L{fkine}, L{tr2diff}, L{jacbo0}, L{ikine560}
    """
     
    #solution control parameters
    
    ilimit = 1000
    stol = 1e-12
    n = robot.n

    #if q == None:
    if q is None:
        q = np.mat(np.zeros((n,1)))
    else:
        q = np.mat(q).flatten().T
        
    #if q != None and m != None:
    if not(q is None) and not(m is None):
        m = np.mat(m).flatten().T
        if len(m)!=6:
            error('Mask matrix should have 6 elements')
        if len(m.nonzero()[0].T)!=robot.n:
            error('Mask matrix must have same number of 1s as robot DOF')
    else:
        if n<6:
            print ('For a manipulator with fewer than 6DOF a mask matrix argument should be specified')
        m = np.mat(np.ones((6,1)))

    if isinstance(tr, list):
        #trajectory case
        qt = np.mat(np.zeros((0,n)))
        for T in tr:
            nm = 1
            count = 0
            while nm > stol:
                e = np.multiply(tr2diff( fkine(robot, q.T), T), m)
                dq = np.linalg.pinv(jacob0(robot,q.T))*e
                q += dq
                nm = np.linalg.norm(dq)
                count += 1
                if count > ilimit:
                    print ('i=',i,'   nm=',nm)
                    error("Solution wouldn't converge")
            qt = vstack( (qt, q.T) )
        return qt;
    elif ishomog(tr):
        #single xform case
        nm = 1
        count = 0
        while nm > stol:
            e = np.multiply( tr2diff(fkine(robot,q.T),tr), m )
            dq = np.linalg.pinv(jacob0(robot, q.T)) * e
            q += dq;
            nm = np.linalg.norm(dq)
            count += 1
            if count > ilimit:
                print ("Solution wouldn't converge, nm = %f"%(nm))
                #error("Solution wouldn't converge")
                break
        qt = q.T
        #print "nm = %f"%(nm)
        return (qt,nm,count)
    else:
        error('tr must be 4*4 matrix')


def jacob0(robot, q):
    """
    Compute manipulator Jacobian in world coordinates for joint coordinates C{q}.

    The manipulator Jacobian matrix maps differential changes in joint space
    to differential Cartesian motion (world coord frame) of the end-effector

    M{dX = J dQ}

    @type robot: Robot
    @type q: vector M{n x 1}
    @param q: joint coordinate
    @rtype: matrix M{6 x n}
    @return: Manipulator Jacobian
    @see: L{jacobn}, L{diff2tr}, L{tr2diff}
    """


    q = np.mat(q)
    Jn = jacobn(robot, q) # Jacobian from joint to wrist space
    #   convert to Jacobian in base coordinates
    Tn = fkine(robot,q) # end-effector transformation
    R = t2r(Tn)
    return np.concatenate( ( np.concatenate( (R,np.zeros((3,3))) ,1) , np.concatenate( (np.zeros((3,3)),R) ,1) ))*Jn


def jacobn(robot, q):
    """
    Compute manipulator Jacobian in tool coordinates for joint coordinates C{q}.

    The manipulator Jacobian matrix maps differential changes in joint space
    to differential Cartesian motion (tool coord frame) of the end-effector.

    M{dX = J dQ}
		
    Reference
    =========
 	
 	Paul, Shimano, Mayer
    Differential Kinematic Control Equations for Simple Manipulators
    IEEE SMC 11(6) 1981
    pp. 456-460

    @type robot: Robot
    @type q: vector M{n x 1}
    @param q: joint coordinate
    @rtype: matrix M{6 x n}
    @return: Manipulator Jacobian
    @see: L{jacobn}, L{diff2tr}, L{tr2diff}
    """
    q = arg2array(q)
    n = robot.n
    L = robot.links
    J = np.mat([[],[],[],[],[],[]])
    U = robot.tool
    for j in range(n-1,-1,-1):
            if not robot.ismdh():    #standard DH convention
                    U = L[j].tr(q[j])*U
            if L[j].sigma == 0: #revolute axis
                    d = np.matrix([[-U[0,0]*U[1,3] + U[1,0]*U[0,3]],\
                                [-U[0,1]*U[1,3] + U[1,1]*U[0,3]],\
                                [-U[0,2]*U[1,3] + U[1,2]*U[0,3]]])
                    delta = U[2,0:3].T   # nz  oz  az
            else: #prismatic axis
                    d = U[2,0:3].T       # nz  oz  az
                    delta = np.zeros((3,1)) # 0   0   0
            J = np.concatenate((np.concatenate((d,delta)),J),1)
            if robot.ismdh(): #modified DH convention
                    U=L[j].tr(q[j])*U
    return J



def tr2jac(t):
    """
    Compute a Jacobian to map differentials motion between frames.
    The M{6x6} Jacobian matrix to map differentials (joint velocity) between 
    frames related by the homogeneous transform C{t}.

    @rtype: matrix M{6 x 6}
    @return: Jacobian matrix
    @see: L{tr2diff}, L{diff2tr}
    """
    t = np.mat(t)
    return np.concatenate((
        np.concatenate((t[0:3,0].T, crossp(t[0:3,3],t[0:3,0]).T),1),
        np.concatenate((t[0:3,1].T, crossp(t[0:3,3],t[0:3,1]).T),1),
        np.concatenate((t[0:3,2].T, crossp(t[0:3,3],t[0:3,2]).T),1),
        np.concatenate((np.zeros((3,3)),t[0:3,0:3].T),1) ))



if __name__ == '__main__':
    P0 = np.array([-1,3.])
    Ph = e2h(P0);
    P1 = h2e(Ph);
    print (P0,Ph,P1)

    print (arg2array(1))
    print (arg2array(1.0))
    print (arg2array( np.mat([1,2,3,4]) ))
    print (arg2array( np.mat([1,2,3,4]).T ))
    print (arg2array( np.array([1,2,3]) ))
    print (arg2array( np.array([1,2,3]).T ))
    print (arg2array( [1,2,3]))
    

    
